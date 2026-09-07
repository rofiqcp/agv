const { test, expect } = require("@playwright/test");
test("rosweb loads", async ({page})=>{await page.goto("/",{waitUntil:"domcontentloaded"}); await expect(page.locator("body")).toBeVisible();});
