@echo off
dotnet tool restore
dotnet paket restore
dotnet build src/Aardvark.Reconstruction.sln