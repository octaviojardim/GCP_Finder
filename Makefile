SHELL := /usr/bin/bash
install:
	python3 -m venv venv
	./venv/bin/pip install -r ./requirements.txt
	sudo apt install libimage-exiftool-perl
activate:
	source ./venv/bin/activate
