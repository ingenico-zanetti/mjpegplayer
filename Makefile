mjpegplayer: mjpegplayer.c
	gcc mjpegplayer.c -o mjpegplayer -O2 -I/usr/local/include -I/usr/include/libdrm/ /usr/local/lib/libturbojpeg.a -ldrm -lpthread -lm
