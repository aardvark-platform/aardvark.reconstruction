namespace Aardvark.Reconstruction.Epipolar.Demo

open System
open Aardvark.Base
open FSharp.Data.Adaptive
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
        //ArbDemo.singleArbTest()
        ArbDemo.manyArbsAndRenderIfBad()
        //Stats.runny()
        //Stats.statty()
        //Stats.runnyAndStatty()

        printfn "Hello World from F#!"
        0 // return an integer exit code