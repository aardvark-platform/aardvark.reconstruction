namespace Aardvark.Reconstruction.Epipolar.Demo

open System
open Aardvark.Base
open Aardvark.Base.Incremental
open Aardvark.Base.Incremental.Operators
open Aardvark.Rendering.Text
open Aardvark.Base.Rendering
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
            ndcOffsetSum : float
            ndcCount : int
            camDiff : float
        }

    type Run =
        {
            i : int
            testKind : string
            noise : float
            pointLayout : string * float
            camTrans : string
            camRot : string
            stats : Option<Stats>
        }

    let runTitleLine =
        "i kind noise layout flatness camTrans camRot ndcDiffSum ndcCt camDiff"

    let runToString (r : Run) =
        let culture = System.Globalization.CultureInfo.InvariantCulture;
        let sb = StringBuilder()
        sb.AppendFormat(culture,"{0} ",r.i) |> ignore
        sb.AppendFormat(culture,"{0} ",r.testKind) |> ignore
        sb.AppendFormat(culture,"{0:0.000000000} ",r.noise) |> ignore
        sb.AppendFormat(culture,"{0} {1:0.00000000} ",(r.pointLayout |> fst),(r.pointLayout |> snd)) |> ignore
        sb.AppendFormat(culture,"{0} ",r.camTrans) |> ignore
        sb.AppendFormat(culture,"{0} ",r.camRot) |> ignore
        match r.stats with 
        | None -> sb.AppendFormat(culture,"     ") |> ignore
        | Some stats -> 
            sb.AppendFormat(culture,"{0:0.00000000} ", stats.ndcOffsetSum) |> ignore
            sb.AppendFormat(culture,"{0} ", stats.ndcCount) |> ignore
            sb.AppendFormat(culture,"{0:0.00000000000}", stats.camDiff) |> ignore
        sb.ToString()        

    let runny () =

        let noiseAmount (scene : Scenario) : float =
            let d = scene |> Scenario.getData
            
            match d.noise with
            | Nope -> 0.0
             // o*0.5 is the average offset of a feature, each feature has 0.5 chance to be offset
            | Offset o -> 0.25 * o 
             // 1.0 is the average offset of a feature, each feature has p chance to be offset
            | Garbage p -> p
            | OffsetAndGarbage (o,p) -> 0.25*o + p


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


        let iter (i : int) =
            let rand = RandomSystem()
            let scenario = Gen.eval 0 (Random.StdGen(rand.UniformInt(),rand.UniformInt())) Lala.genPlaneScenario

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
                            []
                        | Some (fund,_,_) -> 
                            match FundamentalMatrix.decompose fund c0.proj c1.proj [] with
                            | [] -> 
                                []
                            | fs -> 
                                fs |> List.map (fun m -> m * scale)
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
                            []
                        | Some h -> 
                            match Homography.decompose h c0.proj c1.proj [] with
                            | [] -> 
                                []
                            | hs -> 
                                hs |> List.map (fun m -> m * scale)
                    name,recover,scene

            let pMatches =
                Array.zip scene.pts3d scene.matches
                |> Array.map (fun (p3d,(l,r)) -> r,p3d)

            let mots = recover()
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
                    let sim = unsimilarity c scene.cam1
                    Some {ndcOffsetSum = ndcSum; ndcCount = ndcCount; camDiff = sim}

            let stats = fu co name

            let r = 
                { 
                    i           = i
                    testKind    = name
                    noise       = noiseAmount scenario
                    pointLayout = getPointLayout scenario
                    camTrans    = getCamTrans scenario
                    camRot      = getCamRot scenario
                    stats       = stats
                }
            r


        let ct = 1000000
        let file = @"D:\temp2\runny.txt"
        let inline appendLn s = File.AppendAllText(file,s+"\n")
        appendLn runTitleLine

        Log.startTimed "%s" file
        for i in 1..ct do
            let r = iter i
            appendLn (runToString r)
            Report.Progress(float i/float ct)

        Log.stop()
        Log.line "Done"

        ()

    let statty () =
        Log.startTimed "Reading"
        let ls = File.ReadAllLines(@"D:\temp2\runny.txt")
        Log.stop()
        Log.startTimed "Filtering"
        let toks = ls |> Array.map (fun s -> s.Split([|' '|]) |> Array.map(fun s -> s.Trim()))

        let ostr = 
            toks |> Array.filter (fun ts -> 
                ts.[1] = "HOMOGRAPHY" &&
                ts.[3] = "Plane" //&&
                //ts.[4] <> "AlongForward"
            )    |> Array.choose (fun ts -> 
                if ts.[7].IsNullOrEmpty() then
                    None
                else            
                    let sum = ts.[7] |> float
                    let ct = ts.[8] |> int |> float
                    let avg = (sum / ct) |> sprintf "%.9f"
                    [|ts.[2];avg;ts.[9]|] |> String.concat " " |> Some
            ) |> String.concat "\n"
        Log.stop()

        File.writeAllText @"D:\temp2\statty.txt" ostr
        Log.line "Written"