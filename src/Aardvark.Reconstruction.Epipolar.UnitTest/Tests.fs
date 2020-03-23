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
        getBestFittingMot scene.cam0 scene.cam1 scene.pts3d scene.matches mots 
            |> Option.map (fun m -> { (scene.cam0 + m) with proj = scene.cam1.proj })

    let recoverF (scenario : PrettyFundamentalScenario) =
        let f = FundamentalMatrix.recover 1E+10 scenario.fdata.matches
        match f with
        | None -> 
            None
        | Some (fund,_,_) -> 
            Some fund

module Tests =
    open Impl
    let cfg : FsCheckConfig =
        { FsCheckConfig.defaultConfig with 
            maxTest = 20000
            arbitrary = [ typeof<EpipolarArbitraries> ] 
            
        }
    let eps = 1E-3    
    let inline test name t = testPropertyWithConfig cfg name t

    let fundamentalTransposed =
        test "[Fundamental] Transposed is reverse F" (fun (scenario : PrettyFundamentalScenario) -> 
            match recoverF scenario with
            | None -> false
            | Some f ->
                let scene = scenario.fdata
                let c0 = scene.cam0
                let c1 = scene.cam1
                let scale = (c0.Location - c1.Location).Length
                let mots = 
                    match FundamentalMatrix.decompose f.Transposed c1.proj c0.proj [] with
                    | [] -> 
                        []
                    | fs -> 
                        fs |> List.map (fun m -> m * scale)
                let mot = 
                    getBestFittingMot c1 c0 scene.pts3d scene.matches mots 
                    |> Option.map (fun m -> { (c1 + m) with proj = c0.proj })                
                match mot with             
                | None -> false
                | Some cam ->
                    Camera.approxEqual eps c0 cam
        )   

    let camRecovered =
        test "[Epipolar] True camera recovered" (fun (scenario : Scenario) -> 
            match tryRecoverCamera scenario with
            | None -> false  
            | Some cam -> 
                let real = (getData scenario).cam1
                Camera.approxEqual eps cam real
        )

    let reprojectionError =
        test "[Epipolar] Rerprojection error is zero" (fun (scenario : Scenario) -> 
            match tryRecoverCamera scenario with
            | None -> false  
            | Some cam -> 
                let s = getData scenario
                let ms = 
                    Array.zip (s.pts3d) (s.matches |> Array.map snd)
                let repr = Array.sumBy (fun (p,f) -> Vec.distance f (Camera.projectUnsafe p cam)) ms
                if Fun.IsTiny(repr,eps) then true
                else failwith ""
        )    
    
    let algebraicError =
        test "[Epipolar] Algebraic error is zero" (fun (scenario : Scenario) -> 
            let alg = recoverAlgebraicError scenario
            Fun.IsTiny(alg,eps)
        )    

    let epipolarTests =
        testList "Epipolar Tests" [
            camRecovered
            reprojectionError
            algebraicError
        ]

    let fundamentalTests =
        testList "Fundamental Tests" [
            fundamentalTransposed
        ]    

    let allTests =
        testList "All Tests" [
            epipolarTests
            //fundamentalTests
        ]