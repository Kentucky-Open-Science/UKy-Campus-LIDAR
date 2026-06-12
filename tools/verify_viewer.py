#!/usr/bin/env python3
"""Headless runtime check of the viewer (synthetic + missing-data paths).
Run AFTER `python -m http.server <port>` is serving from web/.
Usage: python tools/verify_viewer.py [port]
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path = [p for p in sys.path if os.path.abspath(p or ".") != _HERE]

from playwright.sync_api import sync_playwright

PORT = sys.argv[1] if len(sys.argv) > 1 else "8123"
SHOTS = os.path.join(_HERE, "..", "extracted")


def check(page, url, shot, wait_ms=7000):
    msgs = []
    page.on("console", lambda m: msgs.append(f"[console.{m.type}] {m.text}")
            if m.type in ("error", "warning") else None)
    page.on("pageerror", lambda e: msgs.append(f"[pageerror] {e}"))
    print(f"--- {url}")
    page.goto(url)
    page.wait_for_timeout(wait_ms)
    for sid in ("manifest-status", "terrain-status", "lidar-status", "fps"):
        print(f"  #{sid}: " +
              page.locator("#" + sid).inner_text().replace("\n", " | "))
    page.mouse.move(640, 420)
    page.wait_for_timeout(600)
    print("  #cursor-readout: " +
          page.locator("#cursor-readout").inner_text().replace("\n", " | "))
    path = os.path.join(SHOTS, shot)
    page.screenshot(path=path)
    print(f"  screenshot: {path}")
    for m in msgs:
        print("  " + m)
    return msgs


def main():
    with sync_playwright() as p:
        browser = p.chromium.launch(args=["--use-gl=angle"])
        page = browser.new_page(viewport={"width": 1280, "height": 800})
        errs = check(page, f"http://localhost:{PORT}/?data=data_test",
                     "viewer-test-synthetic.png")
        page2 = browser.new_page(viewport={"width": 1280, "height": 800})
        errs += check(page2, f"http://localhost:{PORT}/",
                      "viewer-test-nodata.png", wait_ms=3500)
        browser.close()
    fatal = [m for m in errs if "pageerror" in m or "console.error" in m]
    print("FATAL ERRORS:" if fatal else "no fatal JS errors")
    for m in fatal:
        print("  " + m)
    return 1 if fatal else 0


if __name__ == "__main__":
    sys.exit(main())
