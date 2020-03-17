namespace Aardvark.Reconstruction.Epipolar.UnitTest

open System
open Aardvark.Base
open FShade
open Aardvark.Reconstruction
open FsCheck
open Aardvark.Reconstruction.Epipolar.Demo
open Lala
open Expecto

module Gen =

    type TestKind =
        | FundamentalTest
        | HomographyTest

    type TestScenarioConfig = 
        {
            pointsAreFlat : bool
            kind : TestKind
        }

    let getData (d : Scenario) =
        match d with
        | FundamentalScenario d -> d
        | HomographyScenario d -> d

    let genHomographyApts (isFlat : bool) cam scale =
        gen {
            let bounds = dataBounds cam scale
            let! apts = 
                if isFlat then
                    gen {
                        let! q = arbQuadFacing45 cam scale 
                        let! t = floatBetween 0.001 0.2
                        return AlmostLinearQuad(q,t) 
                    }                    
                else 
                    gen {
                        let! q = arbQuadFacing45 cam scale             
                        return InQuad q
                    }                     

            return apts
        }    

    let genFundamentalApts (includeFlat : bool) cam scale =
        gen {
            let bounds = dataBounds cam scale
            let! apts = 
                if includeFlat then
                    gen {
                        let! i = Gen.choose(0,1)
                        match i with 
                        | 0 -> 
                            let! t = floatBetween 0.005 0.2
                            return AlmostLinearVolume(bounds,t)
                        | 1 -> 
                            let! t = floatBetween 0.0001 0.2
                            return AlmostFlatVolume(bounds,t)
                        | _ -> 
                            return failwith "unreachable"   
                    }                    
                else 
                    InVolume bounds |> Gen.constant

            return apts
        }    

    let genTestScenario (cfg : TestScenarioConfig) =
        gen {
            let! scale = floatBetween 4.0 8.0
            let! pointCount = intBetween 128 512

            let! p0 = arbV3d Box3d.Unit
            let! fw0 = arbDir
            let t0 = p0 + fw0
            let nf = Trafo3d.FromNormalFrame(p0,fw0)
            let u0 = nf.Backward.C0.XYZ.Normalized
            let cv0 = CameraView.lookAt p0 t0 u0
            let! proj0 = arbProjection
            let cam0 = { view = cv0; proj = proj0 }

            let! apts = 
                match cfg.kind with
                | FundamentalTest -> 
                    genFundamentalApts cfg.pointsAreFlat cam0 scale
                | HomographyTest ->
                    genHomographyApts cfg.pointsAreFlat cam0 scale                

            let! data = genScenarioData pointCount false cam0 scale apts

            let scenario =
                match cfg.kind with
                | FundamentalTest -> FundamentalScenario data
                | HomographyTest -> HomographyScenario data

            return scenario
        }

    let genHomographyTestScenario =
        gen {
            let! flat = Arb.generate<bool>
            let cfg =
                {
                    kind = TestKind.HomographyTest
                    pointsAreFlat = flat
                }
            let! s = genTestScenario cfg
            return cfg,getData s
        }
    
    let genFundamentalTestScenario =
        gen {
            let! flat = Arb.generate<bool>
            let cfg =
                {
                    kind = TestKind.FundamentalTest
                    pointsAreFlat = flat
                }
            let! s = genTestScenario cfg
            return cfg,getData s           
        }

    let toScenario ( (c,d) :TestScenarioConfig * ScenarioData) =
        match c.kind with
        | FundamentalTest -> FundamentalScenario d
        | HomographyTest -> HomographyScenario d

    type EpipolarArbitraries() =

        static member GenerateArbitraryScenario() =
            genFundamentalTestScenario |> Gen.map toScenario |> Arb.fromGen

