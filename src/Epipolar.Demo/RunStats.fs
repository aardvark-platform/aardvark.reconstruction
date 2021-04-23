namespace Aardvark.Reconstruction.Epipolar.Demo

open System
open Aardvark.Base
open FSharp.Data.Adaptive
open Aardvark.Rendering.Text
open Aardvark.Rendering
open Aardvark.SceneGraph
open Aardvark.Application
open Aardvark.Application.Utilities
open Aardvark.Application.Slim
open FShade
open Aardvark.Reconstruction
open FsCheck
open Lala
open System.IO
open System.Text

module Stats =
    type Stats =
        {
            algebErr : float
            ndcOffsetAvg : float
            camDiff : float
        }

    type Run =
        {
            i : int
            testKind : string
            noiseKind : string
            noiseAmount : float
            pointLayout : string * float
            camTrans : string
            camRot : string
            stats : Option<Stats>
        }

    let runTitleLine =
        "i kind noiseKind noiseAmt layout flatness camTrans camRot algebErr ndcDiff camDiff"

    let runToString (r : Run) =
        let culture = System.Globalization.CultureInfo.InvariantCulture;
        let sb = StringBuilder()
        sb.AppendFormat(culture,"{0} ",r.i) |> ignore
        sb.AppendFormat(culture,"{0} ",r.testKind) |> ignore
        sb.AppendFormat(culture,"{0:0} ",r.noiseKind) |> ignore
        sb.AppendFormat(culture,"{0:0.000000000} ",r.noiseAmount) |> ignore
        sb.AppendFormat(culture,"{0} {1:0.00000000} ",(r.pointLayout |> fst),(r.pointLayout |> snd)) |> ignore
        sb.AppendFormat(culture,"{0} ",r.camTrans) |> ignore
        sb.AppendFormat(culture,"{0} ",r.camRot) |> ignore
        match r.stats with 
        | None -> sb.AppendFormat(culture,"     ") |> ignore
        | Some stats -> 
            sb.AppendFormat(culture,"{0:0.00000000000} ", stats.algebErr) |> ignore
            sb.AppendFormat(culture,"{0:0.00000000000} ", stats.ndcOffsetAvg) |> ignore
            sb.AppendFormat(culture,"{0:0.00000000000}", stats.camDiff) |> ignore
        sb.ToString()        

    let noiseAmount (scene : Scenario) : string*float =
        let d = scene |> Scenario.getData
        
        match d.noise with
        | Nope -> "Nope",0.0
         // 0.025 is the average offset of a feature, each feature has c chance to be offset
        | Offset c -> "Noise",c
         // 1.0 is the average offset of a feature, each feature has p chance to be offset
        | Garbage p -> "Mismatch",p
        | OffsetAndGarbage (c,p) -> "NoiseAndMismatch",c + p


    let getPointLayout (scene : Scenario) : string * float=
        let d = scene |> Scenario.getData
        match d.points with
        | AlmostLinearQuad (_,f)        -> "AlmostLinearPlane",f
        | InQuad _                      -> "Plane",1.0
        | AlmostFlatVolume (_,f)        -> "AlmostPlanarVolume",f
        | AlmostLinearVolume (_,f)      -> "AlmostLinearVolume",f
        | InVolume _                    -> "Volume",1.0

    let getCamTrans (scene : Scenario) : string =
        let d = scene |> Scenario.getData
        match d.camtrans with
        | No -> "None"
        | AlongFw _ -> "AlongForward"
        | InPlane _ -> "InImagePlane"
        | ArbitraryDir _ -> "Arbitrary"

    let getCamRot (scene : Scenario) : string =
        let d = scene |> Scenario.getData
        match d.camrot with
        | Not               -> "None"
        | Normal            -> 
            match d.camtrans with
            | No | AlongFw _ -> "None"
            | _ -> "Normal"
        | NormalAndRoll _   -> "Roll"

    let unsimilarity (c0 : Camera) (c1 : Camera) =
        CameraView.sameness c0.view c1.view


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
                        let f = FundamentalMatrix.recover 1E+10 scene.matches
                        match f with
                        | None -> 
                            666.0,[]
                        | Some (fund,_,_) -> 
                            let alg = 
                                scene.matches
                                |> Array.averageBy (fun (l,r) -> 
                                    abs <| Vec.dot (fund * V3d(l,1.0)) (V3d(r,1.0))
                                )
                            match FundamentalMatrix.decompose fund c0.proj c1.proj [] with
                            | [] -> 
                                alg,[]
                            | fs -> 
                                alg,fs |> List.map (fun m -> m * scale)
                    name,recover,scene
                | HomographyScenario scene -> 
                    let name = "HOMOGRAPHY"
                    let recover() =
                        let c0 = scene.cam0
                        let c1 = scene.cam1
                        let scale = (c1.Location - c0.Location).Length
                        let hom = Homography.recover scene.matches
                        match hom with
                        | None -> 
                            666.0,[]
                        | Some h -> 
                            let alg = 
                                scene.matches
                                |> Array.averageBy (fun (l,r) -> 
                                    Vec.length (h.TransformPosProj(l) - r)
                                )
                            match Homography.decompose h c0.proj c1.proj [] with
                            | [] -> 
                                alg,[]
                            | hs -> 
                                alg,hs |> List.map (fun m -> m * scale)
                    name,recover,scene

            let pMatches =
                Array.zip scene.pts3d scene.matches
                |> Array.map (fun (p3d,(l,r)) -> r,p3d)

            let alg,mots = recover()
            let co = 
                getBestFittingMot scene.cam0 scene.cam1 scene.pts3d scene.matches mots 
                |> Option.map (fun m -> { (scene.cam0 + m) with proj = scene.cam1.proj })

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
            let noisekind, noiseAmount = noiseAmount scenario
            let r = 
                { 
                    i           = i
                    testKind    = name
                    noiseKind   = noisekind
                    noiseAmount = noiseAmount
                    pointLayout = getPointLayout scenario
                    camTrans    = getCamTrans scenario
                    camRot      = getCamRot scenario
                    stats       = stats
                }
            r


        let ct = 1000
        let file = sprintf @"D:\temp2\runny_%s.txt" name
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
        let ls = File.ReadAllLines(sprintf @"D:\temp2\runny_%s.txt" name)
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

    //"i kind noiseKind noiseAmt layout flatness camTrans camRot algebErr ndcDiff camDiff"
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
            | 6 -> ["Arbitrary"]
            | 7 -> ["Normal"; "Roll"]
            | _ -> []
        stattyCond "Hom" want 

        Log.line "Runny 2"
        runnyScen "Fun" Lala.genVolumeScenario
        Log.line "Statty 2"
        let want = 
            function 
            | 1 -> ["FUNDAMENTAL"]
            | 6 -> ["Arbitrary"]
            | 7 -> ["Normal"; "Roll"]
            | _ -> []
        stattyCond "Fun" want 

        Log.line "finished"