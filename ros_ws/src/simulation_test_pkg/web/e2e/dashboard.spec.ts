import { test, expect } from "@playwright/test";
test("scenario editor, batch repeats, replay controls, configuration, comparison, and responsive layout", async ({
  page,
  request,
}) => {
  const temporaryName = `Browser validation scenario ${Date.now()}`;
  const errors: string[] = [];
  page.on("pageerror", (error) => errors.push(error.message));
  await page.goto("/");
  await expect(
    page.getByRole("heading", { name: "Runs & replay", exact: true }),
  ).toBeVisible();
  await expect(page.getByText("Local runner connected")).toBeVisible();
  await page.getByRole("button", { name: "New scenario", exact: true }).click();
  const dialog = page.getByRole("dialog");
  await expect(dialog).toBeVisible();
  await dialog.getByLabel("Scenario name").fill(temporaryName);
  await expect(
    dialog.getByLabel("Goal radius (m)", { exact: true }),
  ).toHaveValue("3");
  await expect(
    dialog.getByLabel("Completion radius (m)", { exact: true }),
  ).toHaveValue("5");
  await expect(
    dialog.getByLabel("Extended goal radius (m)", { exact: true }),
  ).toHaveValue("5");
  await dialog.getByLabel("Goal radius (m)", { exact: true }).fill("3.2");
  await dialog.getByLabel("Completion radius (m)", { exact: true }).fill("5.7");
  await dialog
    .getByLabel("Extended goal radius (m)", { exact: true })
    .fill("6.4");
  await dialog.getByLabel("Occupancy grids", { exact: true }).check();
  await dialog.getByLabel("Gazebo simulation", { exact: true }).check();
  await dialog.getByLabel("Cows · linear (m/s)").fill("0.3");
  await dialog.getByLabel("Limit (simulated minutes)").fill("1");
  await dialog
    .getByRole("button", { name: "Save scenario", exact: true })
    .click();
  await expect(dialog).not.toBeVisible();
  const savedScenario = (
    await (await request.get("/api/scenarios")).json()
  ).find((s: { name: string }) => s.name === temporaryName);
  expect(savedScenario.goal).toMatchObject({
    radius_m: 3.2,
    completion_radius_m: 5.7,
    extended_radius_m: 6.4,
  });
  await page
    .getByRole("navigation")
    .getByRole("button", { name: /Scenarios/ })
    .click();
  await expect(
    page.getByRole("heading", { name: temporaryName }),
  ).toBeVisible();
  await page.getByRole("button", { name: "New batch", exact: true }).click();
  await expect(page.getByRole("dialog").getByLabel("Repeats")).toHaveValue("5");
  await page
    .getByRole("dialog")
    .getByRole("button", { name: "Add another scenario" })
    .click();
  await expect(page.getByRole("dialog").getByLabel("Repeats")).toHaveCount(2);
  await page
    .getByRole("dialog")
    .getByRole("button", { name: "Cancel", exact: true })
    .click();
  await page
    .getByRole("navigation")
    .getByRole("button", { name: /Runs & replay/ })
    .click();
  const runs = await (await request.get("/api/runs")).json();
  const recorded = runs
    .filter(
      (run: { status: string; simulation_duration_s?: number }) =>
        ["success", "timed_out", "cancelled"].includes(run.status) &&
        (run.simulation_duration_s || 0) > 10,
    )
    .sort(
      (
        a: { simulation_duration_s: number },
        b: { simulation_duration_s: number },
      ) => b.simulation_duration_s - a.simulation_duration_s,
    )[0];
  if (recorded) {
    await page
      .getByRole("button")
      .filter({ hasText: "#" + recorded.id.slice(0, 8) })
      .click();
    const slider = page.getByRole("slider", {
      name: "Replay time in simulation seconds",
    });
    await slider.fill(
      String(Math.round(recorded.simulation_duration_s * 10) / 20),
    );
    await page.getByLabel("Playback speed").selectOption("4");
    await page.getByLabel("Path trail duration").selectOption("full");
    await page
      .getByRole("button", { name: "Play replay", exact: true })
      .click();
    await expect(
      page.getByRole("button", { name: "Pause replay" }),
    ).toBeVisible();
    await page.getByRole("button", { name: "Pause replay" }).click();
    await page.getByLabel("Detected cows", { exact: true }).uncheck();
    await page
      .getByRole("button", { name: "Configuration", exact: true })
      .click();
    await expect(
      page.getByText("Immutable inputs saved when this run was queued."),
    ).toBeVisible();
    await page.getByText("grid_config.xml", { exact: true }).click();
    await expect(
      page.getByText("<opencv_storage>", { exact: false }).first(),
    ).toBeVisible();
    await page.getByRole("button", { name: "Overview", exact: true }).click();
    await expect(
      page.getByRole("heading", { name: "Movement by entity" }),
    ).toBeVisible();
  }
  const checks = page.getByRole("checkbox", { name: /for comparison/ });
  if ((await checks.count()) >= 2) {
    await checks.nth(0).check();
    await checks.nth(1).check();
    await page.getByRole("button", { name: "Compare (2)" }).click();
    await expect(
      page.getByRole("heading", { name: "Compare experiments" }),
    ).toBeVisible();
    await page.getByRole("button", { name: "Close dialog" }).click();
  }
  await page.screenshot({
    path: "/tmp/herding-lab-desktop.png",
    fullPage: true,
  });
  await page.setViewportSize({ width: 430, height: 900 });
  await page.screenshot({
    path: "/tmp/herding-lab-mobile.png",
    fullPage: true,
  });
  expect(
    await page.evaluate(
      () => document.documentElement.scrollWidth <= window.innerWidth,
    ),
  ).toBeTruthy();
  expect(errors).toEqual([]);
  // Remove only the temporary scenario created by this browser check.
  const scenarios = await (await request.get("/api/scenarios")).json();
  for (const s of scenarios.filter(
    (s: { name: string }) => s.name === temporaryName,
  ))
    await request.delete("/api/scenarios/" + s.id);
});

test.afterEach(async ({ request }) => {
  const scenarios = await (await request.get("/api/scenarios")).json();
  for (const scenario of scenarios.filter((s: { name: string }) =>
    s.name.startsWith("Browser validation scenario"),
  )) {
    await request.delete("/api/scenarios/" + scenario.id);
  }
});
