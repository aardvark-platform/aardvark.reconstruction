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

[<AutoOpen>]
module ArbDemo = 
    let singleArbTest() =

        Ag.initialize()
        Aardvark.Init()
        let win = window {
            backend Backend.GL
        }
        let rand = RandomSystem()

        let counter = Mod.init 0

        let stuff =
            counter |> Mod.map ( fun _ -> 
                Log.startTimed("Generate Scenario")

                let scene = Gen.eval 0 (Random.StdGen(rand.UniformInt(),rand.UniformInt())) Lala.genScenario 

                //let c0, c1, Hpoints2dc0, Hpoints2dc1, Fpoints2dc0, Fpoints2dc1, Hpoints3d, Fpoints3d = Generate.randomScene()
                //let (c0, c1, Hpoints2dc0, Hpoints2dc1, Fpoints2dc0, Fpoints2dc1, Hpoints3d, Fpoints3d) : Camera * Camera * list<V3d * V3d * list<V2d>> * list<V3d * V3d * list<V2d>> * list<V3d * V3d * list<V2d>> * list<V3d * V3d * list<V2d>> * list<V3d * V3d * list<V3d>> * list<V3d * V3d * list<V3d>> = File.readAllBytes @"C:\temp\dump.bin" |> Pickler.pickler.UnPickle
                let c0 = scene.cam0
                let c1 = scene.cam1
                let scale = (c1.Location - c0.Location).Length
                Log.stop()

                Log.startTimed("Calculate epipolar")
                let fMatches = 
                    List.fold2 (fun r (_,_,os0) (_,_,os1) -> (List.zip os0 os1)@r ) [] Fpoints2dc0 Fpoints2dc1
                let hMatches =
                    List.fold2 (fun r (_,_,os0) (_,_,os1) -> (List.zip os0 os1)@r ) [] Hpoints2dc0 Hpoints2dc1
                let p3d = (Hpoints3d@Fpoints3d)
                let pMatches = 
                    List.zip (Hpoints2dc1@Fpoints2dc1) p3d
                    |> List.collect ( fun ((_,_,fs),(_,_,ps)) -> List.zip fs ps)
                    |> List.toArray


                let hom = Homography.recover (hMatches |> List.toArray)
                let hmot =
                    match hom with
                    | None -> 
                        Log.warn "No homography possible"
                        [CameraMotion.Zero]
                    | Some h -> 
                        match Homography.decompose h c0.proj c1.proj [] with //hMatches with
                        | [] -> 
                            Log.warn "Homography decompose failed"
                            [CameraMotion.Zero]
                        | hs -> 
                            Log.line "Decomposed Homography into %d motions" hs.Length
                            hs |> List.map (fun m -> m * scale)
                
                let fund = FundamentalMatrix.recover 1E-6 (fMatches |> List.toArray)
                let fmot =
                    match fund with
                    | None -> 
                        Log.warn "No fundamental possible"
                        [CameraMotion.Zero]
                    | Some (F,lsbr,rsbl) -> 
                        Log.line "lsbr %A" lsbr
                        Log.line "rsbl %A" rsbl
                        match FundamentalMatrix.decompose F c0.proj c1.proj [] with
                        | [] -> 
                            Log.warn "Fundamental decompose failed"
                            [CameraMotion.Zero]
                        | fs -> 
                            Log.line "Decomposed Fundamental into %d motions" fs.Length
                            fs |> List.map (fun m -> m * scale)

                let cp =
                    match P6P.recover pMatches with
                    | None -> 
                        Log.line "P6P failed"
                        c0
                    | Some cp -> 
                        Log.line "Recovered P6P camera"
                        cp

                let cf = { (c0 + (getBestFittingMot c0 c1 fmot)) with proj = c1.proj }
                let ch = { (c0 + (getBestFittingMot c0 c1 hmot)) with proj = c1.proj }

                let fu c = 
                    pMatches |> Array.sumBy (fun (o,p) -> 
                        (c |> Camera.unproject o).GetMinimalDistanceTo(p)
                    )

                if fu ch > 1E-6 then Log.error "fFFFFFFUUUUUUUUUUU ch"
                if fu cf > 1E-6 then Log.error "fFFFFFFUUUUUUUUUUU cf"
                if fu cp > 1E-6 then Log.error "fFFFFFFUUUUUUUUUUU cp"

                
                Log.line "cf %f" (Camera.sameness c1 cf)
                Log.line "ch %f" (Camera.sameness c1 ch)
                Log.line "cp %f" (Camera.sameness c1 cp)

                Log.stop()
                Generate.mkSg p3d,c0,c1,cp,ch,cf,Ppoints2dc0,Hpoints2dc1,Fpoints2dc1,Ppoints2dc1
            )
    
        let sg = Mod.map          (fun (x,_,_,_,_,_,_,_,_,_) -> x) stuff
        let c0 = Mod.map          (fun (_,x,_,_,_,_,_,_,_,_) -> x) stuff
        let c1 = Mod.map          (fun (_,_,x,_,_,_,_,_,_,_) -> x) stuff
        let cp = Mod.map          (fun (_,_,_,x,_,_,_,_,_,_) -> x) stuff
        let ch = Mod.map          (fun (_,_,_,_,x,_,_,_,_,_) -> x) stuff
        let cf = Mod.map          (fun (_,_,_,_,_,x,_,_,_,_) -> x) stuff
        let Ppoints2dc0 = Mod.map (fun (_,_,_,_,_,_,x,_,_,_) -> x) stuff
        let Hpoints2dc1 = Mod.map (fun (_,_,_,_,_,_,_,x,_,_) -> x) stuff
        let Fpoints2dc1 = Mod.map (fun (_,_,_,_,_,_,_,_,x,_) -> x) stuff
        let Ppoints2dc1 = Mod.map (fun (_,_,_,_,_,_,_,_,_,x) -> x) stuff
    
        Log.startTimed("Show")

        let vpo = Mod.map2 (fun (v : Trafo3d[]) (p : Trafo3d[]) -> v.[0] * p.[0]) win.View win.Proj
        let vp0 = c0 |> Mod.map(Camera.viewProjTrafo 0.1 100.0)
        let vp1 = c1 |> Mod.map(Camera.viewProjTrafo 0.1 100.0)
        let vph = ch |> Mod.map(Camera.viewProjTrafo 0.1 100.0)
        let vpf = cf |> Mod.map(Camera.viewProjTrafo 0.1 100.0)
        let vpp = cp |> Mod.map(Camera.viewProjTrafo 0.1 100.0)

        let current = Mod.init 0
        win.Keyboard.DownWithRepeats.Values.Add (fun k -> 
            match k with
            | Keys.Space -> 
                transact (fun _ -> current.Value <- (current.Value + 1)%6)
                match current.Value with 
                | 0 -> Log.line "freefly"
                | 1 -> Log.line "Camera 0 original"
                | 2 -> Log.line "Camera 1 original"
                | 3 -> Log.line "Camera 1 homography"
                | 4 -> Log.line "Camera 1 fundamental"
                | 5 -> Log.line "Camera 1 P6P"
                | _ -> ()
            | Keys.Enter -> 
                transact (fun _ -> counter.Value <- counter.Value + 1)
            | _ -> ()
        )

        let vt = current |> Mod.bind (fun c -> 
            match c with
            | 0 -> vpo
            | 1 -> vp0
            | 2 -> vp1
            | 3 -> vph
            | 4 -> vpf
            | 5 -> vpp
            | _ -> vpo
        )

        let statusText =
            let t = current |> Mod.map (fun c -> 
                match c with 
                | 0 -> "Freefly"
                | 1 -> "LeftCam"
                | 2 -> "True RightCam"
                | 3 -> "Homography"
                | 4 -> "Fundamental"
                | 5 -> "P6P"
                | _ -> "?"
            )
            Sg.text (Font.create "Times New Roman" FontStyle.Regular) C4b.White t
            |> Sg.scale 0.1
            |> Sg.translate -0.9 -0.9 0.0
            |> Sg.viewTrafo ~~Trafo3d.Identity
            |> Sg.projTrafo ~~Trafo3d.Identity

        let sg3d = 
            sg
            |> Sg.dynamic
            |> Sg.viewTrafo vt
            |> Sg.projTrafo (Mod.constant Trafo3d.Identity)

        let ftrSg = 
            Mod.custom (fun t -> 
                let current = current.GetValue(t)

                let Ppoints2dc0 = Ppoints2dc0.GetValue(t)
                let Hpoints2dc1 = Hpoints2dc1.GetValue(t)
                let Fpoints2dc1 = Fpoints2dc1.GetValue(t)
                let Ppoints2dc1 = Ppoints2dc1.GetValue(t)
                let obs = 
                    match current with 
                    | 1 -> Ppoints2dc0
                    | 2 | 5 -> Ppoints2dc1 
                    | 4 -> Fpoints2dc1
                    | 3 -> Hpoints2dc1
                    | _ -> []
                match obs with 
                | [] -> Sg.empty 
                | _ -> 
                    let obs = obs |> List.collect (fun (_,_,v) -> v)
                    let s1 = 
                        let ps = obs|> List.toArray |> Array.map (fun p -> V3d(p.X,p.Y,-1.0))
                        let cs = Array.create ps.Length C4b.Black
                        IndexedGeometry(
                            Mode = IndexedGeometryMode.PointList,
                            IndexedAttributes = SymDict.ofList [
                                DefaultSemantic.Positions, ps :> Array
                                DefaultSemantic.Colors, cs :> Array
                            ]
                        ) |> Sg.ofIndexedGeometry
                          |> Sg.shader {
                            do! DefaultSurfaces.vertexColor
                            do! DefaultSurfaces.pointSprite
                            do! DefaultSurfaces.pointSpriteFragment
                          }
                          |> Sg.uniform "PointSize" (Mod.constant 3.5)
                          |> Sg.depthTest (Mod.constant DepthTestMode.None)
                    let s2 = 
                        let ps = obs |> List.toArray |> Array.map (fun p -> V3d(p.X,p.Y,-1.0))
                        let cs = Array.create ps.Length C4b.White
                        IndexedGeometry(
                            Mode = IndexedGeometryMode.PointList,
                            IndexedAttributes = SymDict.ofList [
                                DefaultSemantic.Positions, ps :> Array
                                DefaultSemantic.Colors, cs :> Array
                            ]
                        ) |> Sg.ofIndexedGeometry
                          |> Sg.shader {
                            do! DefaultSurfaces.vertexColor
                            do! DefaultSurfaces.pointSprite
                            do! DefaultSurfaces.pointSpriteFragment
                          }
                          |> Sg.uniform "PointSize" (Mod.constant 2.0)
                          |> Sg.depthTest (Mod.constant DepthTestMode.None)
                          |> Sg.pass (RenderPass.after "asd" RenderPassOrder.Arbitrary RenderPass.main)
                    Sg.ofList [s1;s2]
            ) |> Sg.dynamic

        let frustSg =
            Mod.custom (fun t -> 
                let current = current.GetValue(t)
                let c0 = c0.GetValue(t)
                let c1 = c1.GetValue(t)
                let ch = ch.GetValue(t)
                let cf = cf.GetValue(t)
                let cp = cp.GetValue(t)
                if current = 0 then
                    [
                        Aardvark.Reconstruction.Epipolar.Demo.Sg.camera ~~0.49 c0 C4b.Blue
                        Aardvark.Reconstruction.Epipolar.Demo.Sg.camera ~~0.49 c1 C4b.Blue
                        Aardvark.Reconstruction.Epipolar.Demo.Sg.camera ~~0.50 ch C4b.Green
                        Aardvark.Reconstruction.Epipolar.Demo.Sg.camera ~~0.51 cf C4b.Red
                        Aardvark.Reconstruction.Epipolar.Demo.Sg.camera ~~0.52 cp (C4b(100,255,255,255))
                    ] |> Sg.ofList
                else Sg.empty
            ) |> Sg.dynamic

        let sg = 
            Sg.ofList [
                frustSg        
                ftrSg
                sg3d
                statusText
            ]

        win.Scene <- sg
        Log.stop()
        win.Run()
        