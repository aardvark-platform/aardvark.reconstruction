namespace Aardvark.Reconstruction.ModelEstimation.Demo

open System
open Aardvark.Base
open FSharp.Data.Adaptive
open Aardvark.Rendering.Text
open Aardvark.Base.Rendering
open Aardvark.SceneGraph
open Aardvark.Application
open Aardvark.Application.Utilities
open Aardvark.Application.Slim
open FShade
open Aardvark.Reconstruction
open FsCheck
open Aardvark.Reconstruction.Epipolar.Demo
open Lala
open System.IO
open System.Text

module Stats =
    open Aardvark.Reconstruction.Epipolar.Demo.Stats

    let runnyScen name (scenarioGen : Gen<Scenario>) =

        let iter (i : int) =
            let rand = RandomSystem()
            let scenario = Gen.eval 0 (Random.StdGen(rand.UniformInt(),rand.UniformInt())) scenarioGen

            let (name,recover,scene) =
                match scenario with
                | FundamentalScenario scene -> 
                    let name = "FUNDAMENTAL"
                    let recover() =
                        let c0 = scene.cam0
                        let c1 = scene.cam1
                        let scale = (c1.Location - c0.Location).Length
                        failwith "todo"
                        // let f = FundamentalMatrix.recover 1E+10 scene.matches
                        // match f with
                        // | None -> 
                        //     666.0,[]
                        // | Some (fund,_,_) -> 
                        //     let alg = 
                        //         scene.matches
                        //         |> Array.averageBy (fun (l,r) -> 
                        //             abs <| Vec.dot (fund * V3d(l,1.0)) (V3d(r,1.0))
                        //         )
                        //     match FundamentalMatrix.decompose fund c0.proj c1.proj [] with
                        //     | [] -> 
                        //         alg,[]
                        //     | fs -> 
                        //         alg,fs |> List.map (fun m -> m * scale)
                    name,recover,scene
                | HomographyScenario scene -> 
                    let name = "HOMOGRAPHY"
                    let recover() =
                        let c0 = scene.cam0
                        let c1 = scene.cam1
                        let scale = (c1.Location - c0.Location).Length
                        let prob = Homography.Ransac.problem c0.proj c1.proj 0.01
                        let cfg = { RansacConfig.expectedRelativeCount = 0.05; RansacConfig.probability = 0.999 }
                        let train = scene.matches |> Array.mapi (fun i (l,r) -> ImagePoint(l,i),ImagePoint(r,i))
                        let test = scene.matches |> Array.mapi (fun i (l,r) -> ImagePoint(l,i),[|ImagePoint(r,i)|])
                        let sol = prob.Solve(cfg,train,test)
                        if sol |> Seq.isEmpty then 666.0,[]
                        else
                            let s = sol |> Seq.head
                            let model = s.model |> Array.map (fun (l,r) -> l.ndc,r.ndc)
                            let alg = 
                                match Homography.recover model with
                                | Some h -> 
                                    scene.matches
                                    |> Array.averageBy (fun (l,r) -> 
                                        Vec.length (h.TransformPosProj(l) - r)
                                    )
                                | None -> 999.0                                
                            match s.value with
                            | [] -> 
                                6969.0,[]
                            | hs -> 
                                alg,hs |> List.map (fun m -> m.motion * scale)
                    name,recover,scene

            let pMatches =
                Array.zip scene.pts3d scene.matches
                |> Array.map (fun (p3d,(l,r)) -> r,p3d)

            let alg,mots = recover()
            let co = getBestFittingMot scene.cam0 scene.cam1 mots |> Option.map (fun m -> { (scene.cam0 + m) with proj = scene.cam1.proj })

            let fu c n = 
                match c with
                | None -> None
                | Some c -> 
                    let ndcSum = 
                        pMatches |> Array.sumBy (fun (o,p) -> 
                            let obs = (c |> Camera.projectUnsafe p)
                            (obs - o) |> Vec.length
                        )
                    let ndcCount = pMatches |> Array.length    
                    let ndcAvg = ndcSum / float ndcCount
                    let sim = unsimilarity c scene.cam1
                    Some {ndcOffsetAvg = ndcAvg; camDiff = sim; algebErr = alg}

            let stats = fu co name
            let nkind, namt = noiseAmount scenario
            let r = 
                { 
                    i           = i
                    testKind    = name
                    noiseKind   = nkind
                    noiseAmount = namt
                    pointLayout = getPointLayout scenario
                    camTrans    = getCamTrans scenario
                    camRot      = getCamRot scenario
                    stats       = stats
                }
            r


        let ct = 5000
        let file = sprintf @"D:\temp2\runny_ransac_%s.txt" name
        let inline appendLn s = File.AppendAllText(file,s+"\n")
        appendLn runTitleLine

        Log.startTimed "%s" file
        for i in 1..ct do
            let r = iter i
            appendLn (runToString r)
            Report.Progress(float i/float ct)

        Log.stop()
        Log.line "Done %s" name

        ()
    
    let runny() = runnyScen "" Lala.genVolumeScenario

    let stattyCond name (cond : int -> list<string>) =
        Log.startTimed "Reading"
        let ls = File.ReadAllLines(sprintf @"D:\temp2\runny_ransac_%s.txt" name)
        Log.stop()
        Log.startTimed "Filtering"
        let toks = ls |> Array.map (fun s -> s.Split([|' '|]) |> Array.map(fun s -> s.Trim()))
        let ostr = 
            toks |> Array.filter (fun ts -> 
                ts |> Array.mapi (fun i s -> 
                    match cond(i) with
                    | [] -> true
                    | cs -> cs |> List.contains s
                ) |> Array.forall id
            )    |> Array.choose (fun ts -> 
                if ts.[8].IsNullOrEmpty() then
                    None
                else            
                    let alg = ts.[8] 
                    let ndcdiff = ts.[9] 
                    let camdiff = ts.[10] 
                    [|ts.[3];alg;ndcdiff;camdiff|] |> String.concat " " |> (fun s -> s.Replace(".",",")) |> Some
            ) |> String.concat "\n"
        Log.stop()

        let path = @"D:\temp2"

        let suffix = 
            List.init toks.[0].Length (fun i -> cond(i)) |> List.concat
            |> String.concat "_"

        let fn = Path.combine [path;sprintf "statty_%s.txt" suffix]
        File.writeAllText fn ostr
        Log.line "Written %s" fn

    //"i kind noise layout flatness camTrans camRot algebErr ndcDiff camDiff"
    let statty () =

        let want = 
            function 
            | 1 -> ["FUNDAMENTAL"]
            | 6 -> ["Arbitrary"]
            | 7 -> ["Normal"; "Roll"]
            | _ -> []

        stattyCond "" want

    let runnyAndStatty() =

        Log.line "Runny 1"
        runnyScen "Hom" Lala.genPlaneScenario
        Log.line "Statty 1"
        let want = 
            function 
            | 1 -> ["HOMOGRAPHY"]
            | 2 -> ["Noise"]
            | _ -> []
        stattyCond "Hom" want 
        Log.line "Statty 2"
        let want = 
            function 
            | 1 -> ["HOMOGRAPHY"]
            | 2 -> ["Mismatch"]
            | _ -> []
        stattyCond "Hom" want 
        Log.line "Statty 2"
        let want = 
            function 
            | 1 -> ["HOMOGRAPHY"]
            | 2 -> ["NoiseAndMismatch"]
            | _ -> []
        stattyCond "Hom" want 

        Log.line "finished"