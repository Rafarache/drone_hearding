import { test, expect } from "@playwright/test";

test("build logs, failed-build gate, retry and automatic reload", async ({
  page,
}) => {
  let state = {
    available: true,
    status: "idle",
    id: "",
    message: "Ready to build",
    log: "",
    code: { branch: "fixture", revision: "123456789", dirty: true },
  };
  await page.route("**/api/build", async (route) => {
    if (route.request().method() === "POST") {
      state = {
        ...state,
        status: "building",
        id: "fixture-build",
        message: "Building ROS packages",
        log: "Starting >>> grid_visualizer_pkg",
      };
    }
    await route.fulfill({ json: state });
  });
  await page.route("**/api/runs", (route) => route.fulfill({ json: [] }));
  await page.goto("/");
  await page
    .getByRole("button", { name: "Build latest code", exact: true })
    .click();
  const dialog = page.getByRole("dialog", { name: "Build latest code" });
  await expect(
    dialog.getByRole("button", { name: "Build and reload" }),
  ).toBeEnabled();
  await dialog.getByRole("button", { name: "Build and reload" }).click();
  await expect(dialog.getByLabel("Build output")).toContainText(
    "Starting >>> grid_visualizer_pkg",
  );
  await expect(
    page.getByRole("button", { name: "New batch", exact: true }),
  ).toBeDisabled();
  state = {
    ...state,
    status: "failed",
    message: "Compiler failed",
    log: "error: fixture compilation error",
  };
  await expect(
    dialog.getByText("Compiler failed", { exact: true }),
  ).toBeVisible();
  await expect(dialog.getByLabel("Build output")).toContainText(
    "fixture compilation error",
  );
  await expect(
    dialog.getByRole("button", { name: "Build and reload" }),
  ).toBeEnabled();
  await page.screenshot({ path: "/tmp/herding-lab-build.png", fullPage: true });
  await dialog.getByRole("button", { name: "Build and reload" }).click();
  const loaded = page.waitForEvent("load");
  state = { ...state, status: "succeeded", message: "Build complete" };
  await loaded;
  await expect(
    page.getByRole("button", { name: "Build latest code", exact: true }),
  ).toBeVisible();
  expect(
    await page.evaluate(() => sessionStorage.getItem("lab-build")),
  ).toBeNull();
});
