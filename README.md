# RCOM Lab 1: Data-Link Protocol

This project sends a file from a transmitter (Tx) to a receiver (Rx) over an RS-232 serial cable, using a Stop-and-Wait data-link protocol.

## Project Structure

- `src/application_layer.c`: splits the file into START, DATA and END packets (Tx) and rebuilds it (Rx).
- `src/link_layer.c`: the data-link protocol (framing, byte stuffing, BCC checks, RR/REJ, timeouts and retransmissions).
- `src/serial_port.c`, `src/main.c`: provided code, not changed.
- `cable/cable.c`: virtual cable for testing on a single computer (not used in the presentation).
- `penguin.gif`: example file to send.
- `docs/`: lab guides.

## How the Protocol Works

1. **Open**: Tx sends SET, Rx answers with UA.
2. **Transfer**: each packet goes in an I frame numbered 0 or 1. Rx answers RR if the frame is good, REJ if the data is corrupted (BCC2 error). A repeated frame is discarded and acknowledged again with RR.
3. **Close**: Tx sends DISC, Rx answers DISC, Tx sends UA.

Frames with a bad header (BCC1 error) are ignored. If Tx gets no answer, it retransmits when the timer runs out. If it gets a REJ, it retransmits right away. Default settings (in `main.c`): timeout of 4 seconds and 3 retries, so 4 attempts in total. Only timeouts count toward this limit, since a REJ proves the receiver is still there.

At the end, Tx prints how many I frames, retransmissions, REJs and timeouts it had. Rx prints how many I frames, duplicates and REJs (BCC2 errors) it had, plus the transfer efficiency.

## Build and Run

The two PCs of the workbench are connected by the serial cable. One runs the transmitter, the other the receiver. Use the port of each PC:

| PC | Serial port |
| --- | --- |
| tux (rack) | `/dev/ttyS0` |
| Desk computer | `/dev/ttyUSB0` |


```
make run_rx RX_SERIAL_PORT=<port>     # on the Rx PC
make run_tx TX_SERIAL_PORT=<port>     # on the Tx PC
make check_files                      # on the Rx PC: compare penguin.gif with penguin-received.gif
```

The port must go after the target as shown. `TX_SERIAL_PORT=<port> make run_tx` does not work, because the Makefile's own value overrides the environment variable.

## Demonstration Tests

### Test 1: File Transfer

1. Start the receiver.
2. Start the transmitter.
3. Wait until the transmission finishes.
4. Check the integrity of the file: `make check_files` on the Rx PC.

Expected: both programs print the statistics with no retransmissions or timeouts, and `diff` says the files are identical.

### Test 2: File Transfer after One Timeout

1. Start the transmitter.
2. Wait about 5 seconds, so the first SET times out.
3. Start the receiver.
4. Wait until the transmission finishes.
5. Check the integrity of the file: `make check_files` on the Rx PC.

Expected: Tx prints `Timeout (1/4)` and `SET retransmitted`, then `UA received` and the transfer continues. The receiver must be started within about 12 seconds, before the last SET is sent. After that, Tx gives up at the 4th timeout (about 16 seconds).

### Test 3: File Transfer without a Receiver

1. Start the transmitter.
2. Do NOT start the receiver.
3. Check what happens.

Expected: Tx sends SET 4 times, one every 4 seconds, then prints `No UA after 4 attempts` and `Could not establish the connection`, and exits.

### Test 4: File Transfer with Retransmissions

1. Start the receiver.
2. Start the transmitter.
3. During the transmission, unplug the serial cable.
4. After a few seconds (less than 10), plug it back in.
5. Check the integrity of the file: `make check_files` on the Rx PC.

Expected: Tx prints timeouts and retransmits the same I frame until the cable is back. The Tx statistics show retransmissions and timeouts greater than 0, and the files are identical. If the cable is pulled in the middle of a frame, Rx may also send a REJ for the broken frame. Tx gives up 16 seconds after the first copy of the frame, so the cable must be back before the last retransmission (12 seconds).

### Test 5: File Transfer with Random Noise

1. Start the receiver.
2. Start the transmitter.
3. During the transmission, add noise to the line with the noise (short-circuit) box on the workbench.
4. After a few seconds, stop the noise.
5. Check the integrity of the file: `make check_files` on the Rx PC.
6. Check whether REJ frames were sent.

Expected: Rx prints lines like `BCC2 error in I0 | -> REJ0 sent`, and Tx prints `REJ0 received` and retransmits. The statistics show `REJ sent (BCC2 errors)` on Rx and `REJ received` on Tx greater than 0. Noise that hits a frame header (`BCC1 error, frame ignored`), breaks an escape sequence or hits an RR causes a timeout instead of a REJ. The files are identical.

### Test 6: File Transfer with Duplicate Frames

1. Start the receiver.
2. Start the transmitter.
3. During the transmission, disconnect only the Rx -> Tx wire, so the RR frames do not reach Tx.
4. Reconnect it after a few seconds (less than 10).
5. Check the integrity of the file: `make check_files` on the Rx PC.

Expected: Rx accepts an I frame but its RR is lost, so Tx times out and sends the same frame again. Rx prints `I0 duplicate, discarded | -> RR1 sent` (or the I1 version), does not write the data twice, and the `Duplicates` count in the Rx statistics is greater than 0. The files are identical.

## Testing without the Lab PCs

On a single computer, the virtual cable connects `/tmp/ttyS10` (Tx) to `/tmp/ttyS11` (Rx). It needs `socat`. Use 3 terminals: `make run_cable`, then `make run_rx`, then `make run_tx`. In the cable console, `off` / `on` disconnect and reconnect the cable, and `ber <value>` (for example `ber 0.0001`, `ber 0` to turn it off) adds random bit errors.
