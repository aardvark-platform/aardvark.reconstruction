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
open FsCheck
module Test = 
    [<EntryPoint>]
    let main argv =
        //fundamentalChecker()
        //singleRenderTest()
        //runManyExamples()
        //runManyAndRenderIfBad()

        
        let res = Gen.eval 0 (Random.StdGen(235235235,56856869)) Lala.genScenario 
        
        Log.line "%A" res

        printfn "Hello World from F#!"
        0 // return an integer exit code