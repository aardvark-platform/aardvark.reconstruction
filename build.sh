#!/bin/bash

if [ ! -f ".paket/paket" ]; then
	echo installing paket
	dotnet tool install Paket --tool-path .paket --version [5.242.2] 
fi



if [ -f boot.fsx ]; then
	echo running boot
	dotnet tool install fake-cli --tool-path .fakecli --version 5.16.0
	.fakecli/fake run boot.fsx
	rm boot.fsx
	rm -dfr .fakecli
fi

if [ ! -f paket.lock ]; then
	.paket/paket install
fi

.paket/paket restore
dotnet packages/build/fake-cli/tools/netcoreapp2.1/any/fake-cli.dll build $@


