#!/bin/sh
# Run ON the Pi once (as a user that can sudo): hook tars_autostart.sh into boot.
#   sh vision/qnx/autostart/install_autostart.sh
# QNX 8 Pi images differ in which user boot script they run. We add one line to
# the first of these that exists; check it with a reboot (see README "Autostart").
HERE="$(cd "$(dirname "$0")" && pwd)"
LINE="sh $HERE/tars_autostart.sh start &"
TAG="# tars-autostart"
for f in /etc/rc.d/rc.local /system/etc/startup/post_startup.sh /data/var/etc/rc.local; do
    if [ -f "$f" ]; then
        if grep -q "$TAG" "$f"; then echo "already installed in $f"; exit 0; fi
        echo "adding to $f:"
        echo "  $LINE $TAG"
        printf '\n%s %s\n' "$LINE" "$TAG" | sudo tee -a "$f" > /dev/null || exit 1
        sudo chmod +x "$f" 2>/dev/null
        echo "done. Reboot, then: sh $HERE/tars_autostart.sh status"
        exit 0
    fi
done
echo "!! no known boot script found (looked for /etc/rc.d/rc.local, /system/etc/startup/post_startup.sh)."
echo "!! Find what your image runs at boot (e.g. grep -r rc.local /etc /system/etc 2>/dev/null)"
echo "!! and add this line to it:"
echo "   $LINE"
exit 1
