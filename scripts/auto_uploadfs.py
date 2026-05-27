#
# auto_uploadfs.py — PlatformIO post-action.
#
# After a USB firmware upload (`pio run -t upload`) completes, also flash
# the LittleFS filesystem image so a fresh/blank device comes up fully
# provisioned in ONE command — no separate `pio run -t uploadfs` step.
#
# Why this exists: `upload` flashes bootloader + partition table + app,
# but NEVER the filesystem (that's deliberately a separate `uploadfs`
# target so routine code uploads don't wipe runtime data). On a brand
# new board the LittleFS partition is unformatted, so scope-snapshot
# storage doesn't work until the fs is initialised. This hook closes
# that gap on first flash.
#
# It chains the existing `uploadfs` target on the SAME env + upload port.
# Only fires on the `upload` target, so `uploadfs` itself can't recurse.
#
# Note: the firmware ALSO calls LittleFS.begin(true) (auto-format on
# mount failure), so a fresh device self-heals even without this. This
# hook just guarantees a clean, pre-formatted fs at flash time.
#
import time

Import("env")  # noqa: F821


def after_firmware_upload(source, target, env):
    print("\n[auto_uploadfs] firmware uploaded → flashing LittleFS image…")
    # The firmware upload just hard-reset the board via RTS and released
    # the serial port. Give the OS a moment to re-enumerate it before the
    # nested uploadfs grabs it again — avoids a port-busy race on
    # back-to-back flashes.
    time.sleep(2)
    py = env.subst("$PYTHONEXE")
    pioenv = env["PIOENV"]
    port = env.subst("$UPLOAD_PORT")

    cmd = '"{py}" -m platformio run -e {env} -t uploadfs'.format(py=py, env=pioenv)
    if port:
        cmd += ' --upload-port "{port}"'.format(port=port)

    rc = env.Execute(cmd)
    if rc:
        # Non-fatal: the firmware itself is already flashed & valid, and
        # LittleFS.begin(true) will format the partition on first boot.
        print("[auto_uploadfs] WARN: fs upload returned {} — firmware is "
              "fine, fs will auto-format on boot.".format(rc))


env.AddPostAction("upload", after_firmware_upload)  # noqa: F821
