namespace Aardvark.Reconstruction.Epipolar.UnitTest

open System
open Aardvark.Base
open FShade
open Aardvark.Reconstruction
open FsCheck
open Expecto
open Expecto.ExpectoFsCheck
open Gen 
open Aardvark.Reconstruction.Epipolar.Demo

module Impl =


    let recoverAlgebraicError (scenario : Scenario) =
        let recover, scene =
            match scenario with
                | FundamentalScenario scene -> 
                    let recover() =
                        let c0 = scene.cam0
                        let c1 = scene.cam1
                        let scale = (c1.Location - c0.Location).Length
                        let f = FundamentalMatrix.recover 1E+10 scene.matches
                        match f with
                        | None -> 
                            666.0
                        | Some (fund,_,_) -> 
                            let alg = 
                                scene.matches
                                |> Array.averageBy (fun (l,r) -> 
                                    abs <| Vec.dot (fund * V3d(l,1.0)) (V3d(r,1.0))
                                )
                            alg                            
                    recover,scene
                | HomographyScenario scene -> 
                    let recover() =
                        let c0 = scene.cam0
                        let c1 = scene.cam1
                        let scale = (c1.Location - c0.Location).Length
                        let hom = Homography.recover scene.matches
                        match hom with
                        | None -> 
                            666.0
                        | Some h -> 
                            let alg = 
                                scene.matches
                                |> Array.averageBy (fun (l,r) -> 
                                    Vec.length (h.TransformPosProj(l) - r)
                                )
                            alg
                    recover,scene

        recover()

    let tryRecoverCamera (scenario : Scenario) =
        let recover, scene =
            match scenario with
                | FundamentalScenario scene -> 
                    let recover() =
                        let c0 = scene.cam0
                        let c1 = scene.cam1
                        let scale = (c1.Location - c0.Location).Length
                        let f = FundamentalMatrix.recover 1E+10 scene.matches
                        match f with
                        | None -> 
                            []
                        | Some (fund,_,_) -> 
                            match FundamentalMatrix.decompose fund c0.proj c1.proj [] with
                            | [] -> 
                                []
                            | fs -> 
                                fs |> List.map (fun m -> m * scale)
                    recover,scene
                | HomographyScenario scene -> 
                    let recover() =
                        let c0 = scene.cam0
                        let c1 = scene.cam1
                        let scale = (c1.Location - c0.Location).Length
                        let hom = Homography.recover scene.matches
                        match hom with
                        | None -> 
                            []
                        | Some h -> 
                            match Homography.decompose h c0.proj c1.proj [] with
                            | [] -> 
                                []
                            | hs -> 
                                hs |> List.map (fun m -> m * scale)
                    recover,scene

        let mots = recover()
        getBestFittingMot scene.cam0 scene.cam1 mots |> Option.map (fun m -> { (scene.cam0 + m) with proj = scene.cam1.proj })

module Tests =
    open Impl
    let cfg : FsCheckConfig =
        { FsCheckConfig.defaultConfig with 
            maxTest = 1000
            arbitrary = [ typeof<EpipolarArbitraries> ] 
        }

    let inline test name t = testPropertyWithConfig cfg name t

    let cam =
        test "[Epipolar] True camera recovered" (fun (scenario : Scenario) -> 
            match tryRecoverCamera scenario with
            | None -> false
            | Some cam -> 
                let real = (getData scenario).cam1
                Camera.approxEqual 1E-6 cam real
        )

    let allTests =
        testList "All Epipolar Tests" [
            cam
        ]    