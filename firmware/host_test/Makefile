#builds the pc test program from the firmware's packet and swing detection code
CFLAGS = -std=gnu11 -Wall -Wextra -Werror -O2 -I../App/Inc
host_sim: host_sim.c ../App/Src/frame.c ../App/Src/swing_detect.c
	$(CC) $(CFLAGS) -o $@ $^
clean:
	rm -f host_sim host_sim.exe
