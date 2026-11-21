# Beat the Goalie

An AI robot goalkeeper for Hack Western 13. An overhead webcam tracks a
ping-pong ball. A QNX real-time loop on the Pi 5 predicts where the ball will
cross the goal line and swings a DC-motor paddle there. An accelerometer
counts the saves.

Protocol and coordinates: [docs/architecture.md](docs/architecture.md).

```
sim/       feasibility + controller sim (run this first: is the motor fast enough?)
pi/        QNX C server goalie_motor (Makefile: make / host / deploy / run)
arduino/   Uno sketch: motor driver, MPU6050 hit detect, 28BYJ-48 needle, flag
vision/    laptop: webcam tracking, scoreboard UI, Switch controller, Claude taunts
docs/      architecture, wiring, RPM test, demo run sheet
```

Quickstart: TODO (filled in after the components land).
