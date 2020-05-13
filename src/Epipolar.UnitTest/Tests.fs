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

    let recoverNormal (scenario : PrettyHomographyScenario) =
        let recover, scene =
            let scene = scenario.hdata
            let recover() =
                let c0 = scene.cam0
                let c1 = scene.cam1
                let scale = (c1.Location - c0.Location).Length
                let hom = Homography.recover scene.matches
                match hom with
                | None -> 
                    []
                | Some h -> 
                    match Homography.decomposeWithNormal h c0.proj c1.proj [] with
                    | [] -> 
                        []
                    | (hs) -> 
                        hs |> List.map (fun (m,n) -> (m * scale,n))
            recover,scene

        let mots = recover()
        getBestFittingMoti scene.cam0 scene.cam1 scene.pts3d scene.matches (mots |> List.map fst) 
            |> Option.map (fun (m,i) -> mots |> List.item i |> snd)


module Tests =
    open Impl
    let cfg : FsCheckConfig =
        { FsCheckConfig.defaultConfig with 
            maxTest = 2000
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
                    getBestFittingMot c1 c0 scene.pts3d (scene.matches |> Array.map (fun (a,b) -> (b,a))) mots 
                    |> Option.map (fun m -> { (c1 + m) with proj = c0.proj })                
                match mot with             
                | None -> false
                | Some cam ->
                    let s = Camera.sameness c0 cam
                    if Camera.approxEqual eps c0 cam then true
                    else failwithf "%f" s
        )   

    let homographyNormal =
        test "[Homography] Correct quad normal recovered" (fun (scenario : PrettyHomographyScenario) -> 
            match recoverNormal scenario with
            | None -> false
            | Some norm -> 
                match scenario.hdata.camtrans with
                | No -> norm.AnyNaN
                | _ -> 
                    let real = 
                        match scenario.hdata.points with
                        | InQuad q -> q.Normal
                        | AlmostLinearQuad (q,_) -> q.Normal
                        | _ -> failwith "unreachable"

                    let r = real.Normalized
                    let vt = Camera.viewTrafo scenario.hdata.cam1

                    let n = (vt.Backward.TransformDir norm).Normalized

                    let good = Fun.ApproximateEquals(r,n,eps) || Fun.ApproximateEquals(r,-n,eps)

                    good
        )

    let camRecovered =
        test "[Epipolar] Correct camera recovered" (fun (scenario : Scenario) -> 
            match tryRecoverCamera scenario with
            | None -> false  
            | Some cam -> 
                let real = (getData scenario).cam1
                Camera.approxEqual (eps*10.0) cam real
        )

    let reprojectionError =
        test "[Epipolar] Rerprojection error is zero" (fun (scenario : Scenario) -> 
            match tryRecoverCamera scenario with
            | None -> false  
            | Some cam -> 
                let s = getData scenario
                let ms = 
                    Array.zip (s.pts3d) (s.matches |> Array.map snd)
                let repr = Array.averageBy (fun (p,f) -> Vec.distance f (Camera.projectUnsafe p cam)) ms
                if Fun.IsTiny(repr,eps) then true
                else failwithf "%f > %f" repr eps
        )    
    
    let algebraicError =
        test "[Epipolar] Epipolar property holds" (fun (scenario : Scenario) -> 
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

    let homographyTests =
        testList "Homography Tests" [
            homographyNormal
        ]    

    let allTests =
        testList "All Tests" [
            epipolarTests
            fundamentalTests
            homographyTests
        ]