Import("env")
import os

# Pull in Espressif's librftest.a (from esp-phy-lib, Apache-2.0, dropped in
# lib/rftest/). Use LINKFLAGS so the archive lands at the end of the link
# command, outside PlatformIO's --start-group/--end-group block.
# --whole-archive forces the linker to keep every object, so the few
# wrapper symbols we actually reference (esp_phy_wifi_tx_tone etc) pull
# their transitive deps (wifiscwout, rftest_init) in with them.

project_dir = env["PROJECT_DIR"]
rftest_path = os.path.join(project_dir, "lib", "rftest", "librftest.a").replace("\\", "/")

env.Append(
    LINKFLAGS=[
        "-Wl,--whole-archive",
        rftest_path,
        "-Wl,--no-whole-archive",
    ],
)
