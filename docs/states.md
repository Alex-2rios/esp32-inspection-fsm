# State reference

Nine states. Every one of them either waits for an input or times out, so the machine can never
sit blocked with a part on the fixture.

| State | Entered when | Leaves when | Timeout |
|---|---|---|---|
| `BOOT` | power on | immediately, after the servo is homed | none |
| `IDLE` | previous cycle finished | the presence sensor goes low | none, it waits forever |
| `PART_DETECTED` | a part appears | immediately, after bumping the counter | none |
| `SETTLING` | part counted | 250 ms have passed, or the part is pulled back out | none |
| `MEASURING` | part settled | 7 valid distance samples collected | 2 s, then `FAULT` |
| `DECIDING` | samples ready | immediately, once the median is compared to the target | none |
| `ACTUATING` | verdict issued | 600 ms, the time the gate needs to swing and return | none |
| `CLEARING` | gate back home | the part leaves the fixture | 5 s, then `FAULT` |
| `FAULT` | a timeout fired | the operator presses the reset button | none, deliberately |
| `ESTOP` | the e-stop input goes low | e-stop released **and** reset pressed | none |

## Why the extra states

`SETTLING` and `CLEARING` are the ones people leave out, and both exist because of things that
went wrong on the bench.

Without `SETTLING`, the ultrasonic sensor fires while the part is still rocking and the reading
is 4 or 5 mm off. A quarter second of doing nothing fixed it completely.

Without `CLEARING`, the machine goes straight back to `IDLE` while the part is still sitting
there, sees it again and counts it twice. It is a rising edge problem, and the fix is to wait
for the sensor to actually release before accepting a new part.

## Measurement

Seven samples 30 ms apart, then the median. The mean was the obvious first choice and it is the
wrong one. An ultrasonic sensor occasionally returns a wild outlier when the echo bounces off
something else, and a single 300 mm reading in a batch of seven drags a mean far enough to flip
a verdict. The median just ignores it.

`SENSOR_OFFSET_MM` is the distance from the sensor face to the empty fixture. Part height is
that offset minus the measured distance, so calibrating the station means measuring an empty
fixture once and writing the number into `config.h`.

## Safety behaviour

The e-stop check runs at the top of `loop()`, before the state machine, so it interrupts any
state including `MEASURING` and `ACTUATING`. It homes the servo, kills the outputs and parks the
machine in `ESTOP`.

Coming back needs two separate actions: release the e-stop, then press reset. Requiring both is
the point. A machine that restarts by itself the moment the button pops back out is how people
get hurt.

`FAULT` has no timeout on purpose. It waits for a human, because both faults that lead there
(no echo, part stuck) mean something physical needs attention.

## Serial output

Every transition is logged with the time spent in the state it left:

```
[   14201] IDLE           -> PART_DETECTED  after 8104 ms
[   14201] PART_DETECTED  -> SETTLING       after 0 ms
[   14452] SETTLING       -> MEASURING      after 251 ms
[   14661] MEASURING      -> DECIDING       after 209 ms
[   14661] DECIDING       -> ACTUATING      after 0 ms
part,12,pass,44.8,45.0,3.0,11,1
[   15262] ACTUATING      -> CLEARING       after 601 ms
[   16104] CLEARING       -> IDLE           after 842 ms
```

Every part also prints one CSV line, so piping the serial monitor to a file gives you a log you
can open in a spreadsheet and check the pass rate without adding any storage to the board.
