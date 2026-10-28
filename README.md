# RCOM Lab 1: Data-Link Protocol

This project sends a file from a transmitter (Tx) to a receiver (Rx) over a serial port, using a Stop-and-Wait data-link protocol.

## Project Structure

- `src/application_layer.c`: splits the file into START, DATA and END packets (Tx) and rebuilds it (Rx).
- `src/link_layer.c`: the data-link protocol (framing, byte stuffing, BCC checks, RR/REJ, timeouts and retransmissions).
- `src/serial_port.c`, `src/main.c`: provided code, not changed.
- `cable/cable.c`: virtual cable that connects `/tmp/ttyS10` (Tx) to `/tmp/ttyS11` (Rx). Needs `socat`.
- `penguin.gif`: example file to send.
- `docs/`: lab guides.

## How the Protocol Works

1. **Open**: Tx sends SET, Rx answers with UA.
2. **Transfer**: each packet goes in an I frame numbered 0 or 1. Rx answers RR if the frame is good, REJ if the data is corrupted (BCC2 error). A repeated frame is discarded and acknowledged again with RR.
3. **Close**: Tx sends DISC, Rx answers DISC, Tx sends UA.

Frames with a bad header (BCC1 error) are ignored. If Tx gets no answer, it retransmits when the timer runs out. Default settings (in `main.c`): timeout of 4 seconds and 3 retries, so 4 attempts in total.

At the end, Tx prints how many I frames, retransmissions, REJs and timeouts it had. Rx prints how many I frames, duplicates and REJs (BCC2 errors) it had, plus the transfer efficiency.

## Build and Run

You need 3 terminals, all in the project root.

```
make                  # build main and cable
make run_cable        # terminal 1: virtual cable
make run_rx           # terminal 2: receiver
make run_tx           # terminal 3: transmitter
make check_files      # compare penguin.gif with penguin-received.gif
```

The cable console accepts these commands:

- `off` / `on`: disconnect / reconnect the cable.
- `ber <value>`: add random bit errors, for example `ber 0.0001`. Use `ber 0` to turn it off.
- `help`: show all commands.

## Demonstration Tests

Before each test, start the cable with `make run_cable` and leave it running.

### Test 1: File Transfer

1. Start the receiver: `make run_rx`
2. Start the transmitter: `make run_tx`
3. Wait until the transmission finishes.
4. Check the integrity of the file: `make check_files`

Expected: both programs print the statistics with no retransmissions, and `diff` says the files are identical.

### Test 2: File Transfer after One Timeout

1. Start the transmitter: `make run_tx`
2. Wait about 5 seconds, so the first SET times out.
3. Start the receiver: `make run_rx`
4. Wait until the transmission finishes.
5. Check the integrity of the file: `make check_files`

Expected: Tx prints `Timeout (1/4)` and `SET retransmitted`, then `UA Received` and the transfer continues. The receiver must be started before the 4th timeout (about 16 seconds).

### Test 3: File Transfer without a Receiver

1. Start the transmitter: `make run_tx`
2. Do NOT start the receiver.
3. Check what happens.

Expected: Tx sends SET 4 times, one every 4 seconds, then prints `No UA after 4 attempts` and `Could not establish the connection`, and exits.

### Test 4: File Transfer with Retransmissions

1. Start the receiver: `make run_rx`
2. Start the transmitter: `make run_tx`
3. During the transmission, type `off` in the cable console.
4. After a few seconds (less than 16), type `on`.
5. Check the integrity of the file: `make check_files`

Expected: Tx prints timeouts and retransmits the same I frame until the cable is back on. The Tx statistics show retransmissions and timeouts greater than 0, and the files are identical.

### Test 5: File Transfer with Random Noise

1. Start the receiver: `make run_rx`
2. Start the transmitter: `make run_tx`
3. During the transmission, type `ber 0.0001` in the cable console.
4. After a few seconds, type `ber 0`.
5. Check the integrity of the file: `make check_files`
6. Check whether REJ frames were sent.

Expected: Rx prints lines like `BCC2 error in I0, REJ0 sent` and Tx prints `REJ0 received` and retransmits. The statistics show `REJ sent (BCC2 errors)` on Rx and `REJ received` on Tx greater than 0. Noise that hits a frame header causes a timeout instead of a REJ. The files are identical.

### Test 6: File Transfer with Duplicate Frames

1. Start the receiver: `make run_rx`
2. Start the transmitter: `make run_tx`
3. During the transmission, disable the Rx -> Tx channel so the RR frames do not reach Tx.
4. Re-enable the Rx -> Tx channel after a few seconds.
5. Check the integrity of the file: `make check_files`

Expected: Rx accepts an I frame but its RR is lost, so Tx times out and sends the same frame again. Rx prints `Duplicate I0, discarded, RR1 sent` (or the I1 version), does not write the data twice, and the `Duplicates` count in the Rx statistics is greater than 0. The files are identical.

Note: on the real serial cable in the lab, this is done by disconnecting only the Rx -> Tx wire. The virtual cable only has `off`, which cuts both directions. With it, duplicates happen when an I frame reaches Rx just before `off`, so its RR is lost. It may take a couple of tries to get the timing right.
