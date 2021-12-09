SHELL := /usr/bin/bash

.ONESHELL:

.PHONY: install
install:
	python3 -m venv venv
	./venv/bin/pip install -r ./requirements.txt
	apt install exiftool

run:
	source ./venv/bin/activate
	flask run

clean:
	rm -r env



