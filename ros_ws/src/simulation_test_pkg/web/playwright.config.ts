import { defineConfig } from "@playwright/test";
export default defineConfig({
  testDir: "./e2e",
  timeout: 30000,
  use: {
    baseURL: process.env.LAB_URL || "http://127.0.0.1:8765",
    headless: true,
    launchOptions: {
      executablePath: process.env.CHROME_BIN || "/usr/bin/google-chrome",
      args: ["--no-sandbox"],
    },
    viewport: { width: 1440, height: 1100 },
  },
  reporter: "list",
  outputDir: "/tmp/herding-lab-browser-results",
});
