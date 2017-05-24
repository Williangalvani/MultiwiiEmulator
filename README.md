MultiwiiEmulator
=============
This is a badly done program to mimic a Multiwii flight controller, using the [Multiwii Serial Potocol][2].
However the communication is not well done, it doesnt read any answers, just spits messages every X milliseconds.

Setup
------
* Ubuntu -  *`sudo apt-get install socat python-serial`*
	1. socat
	2. python-serial

Instalation
----------

* Manual
	1. [Download][4] the Master branch from gitHub.
	2. Unzip.
	3. Execute socat (`sudo sh socat.sh`)
	4. Execute the program (`python main.py`)

* Git
	1. Download the repository (`git clone https://github.com/Williangalvani/MultiwiiEmulator.git`)
	2. Go inside the folder (`cd MultiwiiEmulator`)
	3. Execute socat (`sudo sh socat.sh`)
	4. Execute the program (`python main.py`)

How to use
-------
The MultwiiEmulator will send data on /dev/ttyVirtual1, so connect your program with /dev/ttyVirtual2.

You can use the [Willian's][24] [MultiwiiEmulator][5] with the [proVANT-groundstation][3] to visualize the data.

Contributing
------------

1. Fork it.
2. Create a branch (`git checkout -b my_markup`)
3. Commit your changes (`git commit -am "Added something very cool"`)
4. Push to the branch (`git push origin my_markup`)
5. Open a [Pull Request][1]
6. Enjoy a good book and wait

Authors
------------
1. [Willian G.][24]
2. [Patrick J.P][77]

[1]: https://github.com/Williangalvani/MultiwiiEmulator/pulls
[2]: https://multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol
[3]: https://github.com/patrickelectric/provant-groundstation
[4]: https://github.com/Williangalvani/MultiwiiEmulator/archive/master.zip
[5]: https://github.com/Williangalvani/MultiwiiEmulator
[77]: https://github.com/patrickelectric
[24]: https://github.com/Williangalvani