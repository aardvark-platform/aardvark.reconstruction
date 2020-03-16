namespace Aardvark.Reconstruction.Epipolar.UnitTest

open System
open Aardvark.Base
open FShade
open Aardvark.Reconstruction
open FsCheck
open Expecto

module Test = 
    [<EntryPoint>]
    let main argv =
        let i = runTests defaultConfig Tests.allTests

        printfn "Hello World from F#! %d" i
        0 // return an integer exit code