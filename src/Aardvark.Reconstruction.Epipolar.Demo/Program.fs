namespace Aardvark.Reconstruction.Epipolar.Demo

open System
open Aardvark.Base
open Aardvark.Base.Incremental
open Aardvark.Base.Incremental.Operators
open Aardvark.Rendering.Text
open Aardvark.Base.Rendering
open Aardvark.SceneGraph
open Aardvark.Application
open Aardvark.Application.Slim
open FShade
open Aardvark.Reconstruction

module Test = 
    [<EntryPoint>]
    let main argv =
        fundamentalChecker()
        //singleRenderTest()
        //runManyExamples()
        //runManyAndRenderIfBad()

        printfn "Hello World from F#!"
        0 // return an integer exit code