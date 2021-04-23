namespace Aardvark.Reconstruction.ModelEstimation.Demo

open System
open Aardvark.Base
open FSharp.Data.Adaptive
open Aardvark.Rendering.Text
open Aardvark.Rendering
open Aardvark.SceneGraph
open Aardvark.Application
open Aardvark.Application.Slim
open FShade
open Aardvark.Reconstruction
open FsCheck
module Test = 
    [<EntryPoint>]
    let main argv =
        ArbDemo.singleArbTest()
        //Stats.runny()
        //Stats.statty()
        //Stats.runnyAndStatty()

        printfn "Hello World from F#!"
        0 // return an integer exit code