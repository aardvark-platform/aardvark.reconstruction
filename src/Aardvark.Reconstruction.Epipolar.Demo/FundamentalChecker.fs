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

[<AutoOpen>]
module Testy =

    let getBestFittingMot (c0 : Camera) (c1 : Camera) (mots : list<CameraMotion>) =
        mots |> List.maxBy (fun mot -> 
            let r = c0 + mot
            let dir = (1.0 + Vec.dot c1.Forward r.Forward)/2.0 * 1000.0
            let pos = -Vec.length (c1.Location - r.Location)
            (dir, pos)
        )

    let fundamentalChecker() =
        Aardvark.Init()
        Ag.initialize()
        
        let win = window {
            backend Backend.GL
            samples 8
        }

        let box = Box3d(V3d.NNN,V3d.III)
        let ftrCt = 5
        let ftrVol (min : V3d) (max : V3d) =
            [|
                let sx = (max.X - min.X) / float ftrCt
                let sy = (max.Y - min.Y) / float ftrCt
                let sz = (max.Z - min.Z) / float ftrCt
                for i in 1..ftrCt do
                    let cx = min.X + float i * sx
                    for j in 1..ftrCt do
                        let cy = min.Y + float j * sy
                        for k in 1..ftrCt do
                            let cz = min.Z + float k * sz
                            yield V3d(cx,cy,cz)
            |]
        let ftrs = ftrVol box.Min box.Max

        let FMode = Mod.init 0
        let speed = Mod.init 1.0
        let focal = Mod.init 1.0
        let fov = focal |> Mod.map (fun focal -> (2.0 * atan( 1.0/focal )) * Constant.DegreesPerRadian)
        let aspect = win.Sizes |> Mod.map (fun s -> float s.X / float s.Y)
        let cv = CameraView.LookAt((V3d.IOO*6.0),V3d.OOO,V3d.OOI)
                    |> DefaultCameraController.controlWithSpeed speed win.Mouse win.Keyboard win.Time
        let frust = 
            Mod.custom (fun t -> 
                let aspect = aspect.GetValue(t)
                let fov = fov.GetValue(t)
                Frustum.perspective fov 0.1 100.0 aspect
            )

        let rview = 
            cv |> Mod.map (fun acv -> 
                CameraView.lookAt acv.Location (acv.Location + acv.Forward) V3d.OOI
            )
        let rproj = 
            Mod.custom (fun t ->
                let aspect = aspect.GetValue(t)
                let focal = focal.GetValue(t)
                let s = win.Sizes.GetValue(t)
                { 
                    focalLength = focal
                    aspect = aspect
                    distortion = {
                        principalPoint = V2d.Zero
                        imageSize = s
                        distortion = RadialDistortion2d.Identity
                    }
                }
            )
        let c1 = 
            Mod.map2 (fun v p ->
                { view = v; proj = p }
            ) rview rproj
        let c0 = Mod.init (c1.GetValue())

        let rand = RandomSystem()
        let noise (p : V2d) =
            let n = rand.UniformV2dDirection() * (rand.UniformDouble()*0.0025)
            p+n
            

        let c0obs =
            Mod.map (fun c0  -> 
                (ftrs |> Array.map (fun p -> Camera.projectUnsafe p c0))
            ) c0

        let c1obs =
            Mod.map (fun c1 -> 
                (ftrs |> Array.map (fun p -> Camera.projectUnsafe p c1))
            ) c1

        let matches = 
            Mod.map2 Array.zip c0obs c1obs

        let sameness = 
            Mod.map2 (fun real ref -> 
                Camera.sameness real ref
            ) c0 c1

        let Fres =
            Mod.custom (fun t -> 
                let FMode = FMode.GetValue t
                if FMode = 0 then
                    let c0 = c0.GetValue t
                    let c1 = c1.GetValue t
                    FundamentalMatrix.fromCameras c0 c1
                else
                    let matches = matches.GetValue t
                    match FundamentalMatrix.recover Double.PositiveInfinity matches with
                    | Some (F,lsbr,rsbl) -> F,lsbr,rsbl
                    | None -> M33d.Identity, V2d.OO, V2d.OO
            )

        let F = Fres |> Mod.map(fun (F,_,_) -> F)        
        let lsbr = Fres |> Mod.map(fun (_,lsbr,_) -> lsbr)
        let rsbl = Fres |> Mod.map(fun (_,_,rsbl) -> rsbl)

        let scale =
            Mod.map2 (fun (c0 : Camera) (c1 : Camera) -> 
                (c1.Location - c0.Location).Length
            ) c0 c1

        let mot =
            Mod.custom (fun t -> 
                let F = F.GetValue(t)
                let c0 = c0.GetValue(t)
                let c1 = c1.GetValue(t)
                let scale = scale.GetValue(t)
                match FundamentalMatrix.decompose F c0.proj c1.proj [] with
                | [] -> None
                | mots -> getBestFittingMot c0 c1 mots |> Some
            )
        let c1e = 
            Mod.custom (fun t -> 
                let c0 = c0.GetValue(t)
                let c1 = c1.GetValue(t)
                let mot = mot.GetValue(t)
                match mot with 
                | Some mot -> 
                    let scale = scale.GetValue(t)
                    { (c0 + mot*scale) with proj = c1.proj }
                | None -> 
                    c0
            )

        let riteText =
            Mod.custom (fun t -> 
                let focal = focal.GetValue(t)
                let fov = fov.GetValue(t)
                let frust = frust.GetValue(t)
                let s = win.Sizes.GetValue(t)
                let c1 = c1.GetValue(t)

                let aspect = frust |> Frustum.aspect
                String.concat "\n" [
                    sprintf "focal %.3f = fov %.1f°" focal fov
                    sprintf "aspect %.5f" aspect
                    sprintf "sizes [%d,%d]" s.X s.Y
                    sprintf "pos [%.2f,%.2f,%.2f]" c1.Location.X c1.Location.Y c1.Location.Z
                    sprintf "fwd [%.3f,%.3f,%.3f]" c1.Forward.X c1.Forward.Y c1.Forward.Z
                ]
            )

        let inline spaces (F : M33d) =
            let longest = 
                [F.M00; F.M01; F.M02; F.M10; F.M11; F.M12; F.M20; F.M21; F.M22]
                |> List.map int
                |> List.map string
                |> List.map String.length
                |> List.max
            fun (f : float) -> 
                let len = f |> int |> string |> String.length
                let num = longest - len + 1
                List.replicate num " " |> String.concat ""

        let inline printMat (F : M33d) =
            let sp = spaces F
            String.concat "\n" [
                sprintf " |%.6f%s%.6f%s%.6f|" F.M00 (sp F.M10) F.M10 (sp F.M20) F.M20
                sprintf " |%.6f%s%.6f%s%.6f|" F.M01 (sp F.M11) F.M11 (sp F.M21) F.M21
                sprintf " |%.6f%s%.6f%s%.6f|" F.M02 (sp F.M12) F.M12 (sp F.M22) F.M22
            ]

        let topText =
            Mod.custom (fun t -> 
                let FMode = FMode.GetValue(t)
                let FStr = 
                    if FMode = 0 then
                        String.concat "\n" [
                            "-> (1) F from cameras <-"
                            "(2) F from features"
                        ]
                    else
                        String.concat "\n" [
                            "(1) F from cameras"
                            "-> (2) F from features <-"
                        ]                    
                FStr                
            )
        let leftText =
            Mod.custom (fun t -> 
                let s = sameness.GetValue(t)
                let F = F.GetValue(t)
                let lsbr = lsbr.GetValue(t)
                let rsbl = rsbl.GetValue(t)
                let mot = mot.GetValue(t)
                let c1 = c1.GetValue(t)
                let c1e = c1e.GetValue(t)
                let correct = Camera.sameness c1 c1e
                let decompStr =
                    match mot with 
                    | None -> "No."
                    | Some mot -> 
                        let R : M33d = mot.trafo.Rot |> Rot3d.op_Explicit
                        let t = mot.trafo.Trans
                        String.concat "\n" [
                            sprintf "decomposed"
                            sprintf "R\n%s" (printMat R)
                            sprintf "t [%.3f,%.3f,%.3f]" t.X t.Y t.Z
                        ]

                String.concat "\n" [
                    sprintf "sameness %.5f" s
                    sprintf "F\n%s" (printMat F)
                    sprintf "lsbr[%.3f,%.3f]  rsbl[%.3f,%.3f]" lsbr.X lsbr.Y rsbl.X rsbl.Y
                    sprintf "%s" decompStr
                    sprintf "similarity %.5f" correct
                ]
            )

        win.Keyboard.DownWithRepeats.Values.Add (fun k ->
            match k with 
            | Keys.PageUp ->   transact(fun _ -> speed.Value <- speed.Value * 1.3)
            | Keys.PageDown -> transact(fun _ -> speed.Value <- speed.Value / 1.3)
            | Keys.OemPlus ->  transact(fun _ -> focal.Value <- focal.Value * 1.025)
            | Keys.OemMinus -> transact(fun _ -> focal.Value <- focal.Value / 1.025)
            | Keys.Enter -> transact(fun _ -> c0.Value <- c1.GetValue())
            | Keys.D1 -> transact (fun _ -> FMode.Value <- 0)
            | Keys.D2 -> transact (fun _ -> FMode.Value <- 1)
            | _ -> ()
        )

        let boxsg =
            Sg.box' C4b.Red box  
            |> Sg.diffuseTexture DefaultTextures.checkerboard
            |> Sg.shader {
                do! DefaultSurfaces.trafo
                do! DefaultSurfaces.diffuseTexture
                do! DefaultSurfaces.simpleLighting
            }

        let leftTextSg =
            Sg.text (Font.create "Times New Roman" FontStyle.Regular) C4b.White leftText
            |> Sg.scale 0.025
            |> Sg.translate -0.99 0.4 0.0
            |> Sg.viewTrafo ~~Trafo3d.Identity
            |> Sg.projTrafo ~~Trafo3d.Identity

        let riteTextSg =
            Sg.text (Font.create "Times New Roman" FontStyle.Regular) C4b.White riteText
            |> Sg.scale 0.025
            |> Sg.translate 0.49 0.4 0.0
            |> Sg.viewTrafo ~~Trafo3d.Identity
            |> Sg.projTrafo ~~Trafo3d.Identity

        let topTextSg =
            Sg.text (Font.create "Times New Roman" FontStyle.Regular) C4b.White topText
            |> Sg.scale 0.03
            |> Sg.translate -0.89 0.9 0.0
            |> Sg.viewTrafo ~~Trafo3d.Identity
            |> Sg.projTrafo ~~Trafo3d.Identity

        let referenceCamSg =
            c0 |> Mod.map (fun cam -> 
                Aardvark.Reconstruction.Epipolar.Demo.Sg.camera ~~0.49 cam C4b.Blue
            ) |> Sg.dynamic

        let ftrSg1 =
            Mod.custom (fun t -> 
                let FMode = FMode.GetValue(t)
                if FMode = 0 then Sg.empty
                else
                    let obs = c1obs.GetValue(t)
                    let s1 = 
                        let ps = obs |> Array.map (fun p -> V3d(p.X,p.Y,-1.0))
                        let cs = Array.create ps.Length C4b.Green
                        IndexedGeometry(
                            Mode = IndexedGeometryMode.PointList,
                            IndexedAttributes = SymDict.ofList [
                                DefaultSemantic.Positions, ps :> Array
                                DefaultSemantic.Colors, cs :> Array
                            ]
                        ) |> Sg.ofIndexedGeometry
                          |> Sg.uniform "PointSize" (Mod.constant 3.5)
                    s1
            ) |> Sg.dynamic

        let ftrSg2 =
            Mod.custom (fun t -> 
                let FMode = FMode.GetValue(t)
                if FMode = 0 then Sg.empty
                else
                    let obs = c1obs.GetValue(t)
                    let s2 = 
                        let ps = obs |> Array.map (fun p -> V3d(p.X,p.Y,-1.0))
                        let cs = Array.create ps.Length C4b.Red
                        IndexedGeometry(
                            Mode = IndexedGeometryMode.PointList,
                            IndexedAttributes = SymDict.ofList [
                                DefaultSemantic.Positions, ps :> Array
                                DefaultSemantic.Colors, cs :> Array
                            ]
                        ) |> Sg.ofIndexedGeometry
                          |> Sg.uniform "PointSize" (Mod.constant 2.0)
                          |> Sg.pass (RenderPass.after "asd" RenderPassOrder.Arbitrary RenderPass.main)
                    s2
            ) |> Sg.dynamic

        let ftrSg =
            Sg.ofList [ftrSg1; ftrSg2]
              |> Sg.shader {
                do! DefaultSurfaces.vertexColor
                do! DefaultSurfaces.pointSprite
                do! DefaultSurfaces.pointSpriteFragment
              }
              |> Sg.depthTest (Mod.constant DepthTestMode.None)

        let sg =
            Sg.ofList [
                Sg.ofList [
                    boxsg
                    ftrSg
                    referenceCamSg
                ]   
                    //|> Sg.viewTrafo (cv |> Mod.map Aardvark.Base.CameraView.viewTrafo)
                    //|> Sg.projTrafo (frust |> Mod.map Frustum.projTrafo)
                    |> Sg.viewTrafo (c1e |> Mod.map (Camera.viewProjTrafo 0.1 100.0))
                    |> Sg.projTrafo (Mod.constant Trafo3d.Identity)
                leftTextSg
                riteTextSg
                topTextSg
            ]

        win.Scene <- sg
        win.Run()
