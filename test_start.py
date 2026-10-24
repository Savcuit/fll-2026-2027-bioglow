# Minimal hardware test — paste into a NEW Pybricks program and run it.
# Expected: red light for 1 s, then green, and the number 1 on the display.
# Press LEFT / RIGHT to change the number, CENTER to exit.
# If the display stays blank here too, the fault is not in the mission code.

from pybricks.hubs import PrimeHub
from pybricks.parameters import Color, Button
from pybricks.tools import wait

hub = PrimeHub()

hub.light.on(Color.RED)
wait(1000)
hub.light.on(Color.GREEN)

n = 1
hub.display.number(n)

while True:
    pressed = hub.buttons.pressed()
    if pressed:
        wait(200)
        if Button.LEFT in pressed:
            n = n - 1 if n > 0 else 3
        elif Button.RIGHT in pressed:
            n = n + 1 if n < 3 else 0
        elif Button.CENTER in pressed:
            hub.display.off()
            hub.light.off()
            break
        hub.display.number(n)
    wait(20)
