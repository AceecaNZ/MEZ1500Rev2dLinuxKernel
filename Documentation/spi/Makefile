CROSS=arm-linux-

#INCPATH=/opt/Aceeca/Linux/MEZ1500/MEZ1500Rev2dLinux/Sources/linux-2.6.32.2/include

all: spidev_test

spidev_test: spidev_test.c
#	$(CROSS)gcc -I $(INCPATH) -o spidev_test spidev_test.c
	$(CROSS)gcc -o spidev_test spidev_test.c

clean:
	@rm -vf spidev_test *.o *~
