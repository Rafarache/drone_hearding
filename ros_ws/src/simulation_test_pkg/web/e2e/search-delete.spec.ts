import { test, expect } from "@playwright/test";
test("search layer toggles and single/bulk experiment deletion", async ({
  page,
  request,
}) => {
  const scenario = await (await request.get("/api/defaults")).json();
  let runs = ["a", "b", "c"].map((key, index) => ({
    id: key.repeat(32),
    batch_id: "f".repeat(32),
    batch_name: "Browser-only fixtures",
    created_at: new Date().toISOString(),
    scenario: { ...scenario, name: "Deletion fixture " + index },
    status: "success",
    repetition: 1,
    simulation_duration_s: 10,
    real_duration_s: 12,
    cancel_requested: false,
    entities: [
      { name: "/simple_drone0", kind: "drone", index: 0 },
      { name: "/simple_drone1", kind: "drone", index: 1 },
    ],
  }));
  const deleted: string[][] = [];
  await page.route("**/api/batches", (route) => route.fulfill({ json: [] }));
  await page.route("**/api/runs**", async (route) => {
    const path = new URL(route.request().url()).pathname;
    if (path === "/api/runs") return route.fulfill({ json: runs });
    if (path === "/api/runs/delete") {
      const ids = route.request().postDataJSON().run_ids;
      deleted.push(ids);
      runs = runs.filter((run) => !ids.includes(run.id));
      return route.fulfill({ json: { deleted: ids } });
    }
    if (path.endsWith("/search"))
      return route.fulfill({
        json: {
          available: true,
          drones: [0, 1].map((index) => ({
            drone: "/simple_drone" + index,
            first_t: 0,
            grid: { width: 4, height: 4, resolution: 10, origin: [-20, -20] },
            cells: [
              [index, 0],
              [index + 4, 5],
            ],
            goals: [
              {
                t: 0,
                goal: {
                  viewpoint: [index, 0],
                  patch_center: [5, 5],
                  patch: [
                    [4, 4],
                    [6, 4],
                    [6, 6],
                    [4, 6],
                  ],
                  strength: 0.6,
                },
              },
            ],
          })),
        },
      });
    if (path.endsWith("/summary"))
      return route.fulfill({
        json: { entities: [], cows_at_goal: 0, missing_entities: [] },
      });
    if (path.endsWith("/trajectory"))
      return route.fulfill({ json: { frames: [], detections: [] } });
    if (path.endsWith("/visualizations"))
      return route.fulfill({
        json: {
          available: false,
          settings: scenario.visualization,
          applied: false,
          errors: [],
        },
      });
    return route.fulfill({ json: {} });
  });
  page.on("dialog", (dialog) => dialog.accept());
  await page.goto("/");
  await expect(
    page.getByRole("heading", { name: "Deletion fixture 0", exact: true }),
  ).toBeVisible();
  await page.getByText("Search map & goals", { exact: false }).click();
  await expect(
    page.getByLabel("Searched areas", { exact: true }),
  ).toBeChecked();
  const canvas = page.locator(".map-wrap canvas");
  const before = await canvas.evaluate((element) =>
    (element as HTMLCanvasElement).toDataURL(),
  );
  await page.getByLabel("Searched areas", { exact: true }).uncheck();
  await expect
    .poll(() =>
      canvas.evaluate((element) => (element as HTMLCanvasElement).toDataURL()),
    )
    .not.toBe(before);
  await page.getByLabel("Unsearched areas", { exact: true }).uncheck();
  await page.getByLabel("Search goals", { exact: true }).uncheck();
  await page.getByRole("checkbox", { name: /^D0/ }).uncheck();
  await page
    .getByRole("button", { name: "Delete experiment", exact: true })
    .click();
  await expect(
    page.getByRole("heading", { name: "Deletion fixture 1", exact: true }),
  ).toBeVisible();
  expect(deleted[0]).toEqual(["a".repeat(32)]);
  await page
    .getByRole("checkbox", { name: "Select all filtered experiments" })
    .check();
  await page
    .getByRole("button", { name: "Delete selected (2)", exact: false })
    .click();
  await expect.poll(() => runs.length).toBe(0);
  expect(deleted[1].sort()).toEqual(["b".repeat(32), "c".repeat(32)]);
});
