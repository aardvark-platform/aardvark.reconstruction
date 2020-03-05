namespace Test

open System
open Aardvark.Base
open FSharp.Data.Adaptive
open MiniCV 
open Aardvark.Geometry
open Aardvark.Base.Rendering
open Aardvark.SceneGraph
open Aardvark.Application.Slim
open Aardvark.Application
open Aardvark.Application.Utilities

module Bla =

    let heat =
        let heaty =
            [|
                C4f.Blue
                C4f.Cyan
                C4f.Green
                C4f.Yellow
                C4f.Red
            |]

        fun (v : float) ->
            let v = clamp 0.0 1.0 v

            let v = v * 4.0
            let i = v |> floor |> int
                
            let l = heaty.[i]
            let r = if i < heaty.Length - 1 then heaty.[i + 1] else l
            let frac = v - float i
            ((1.0 - frac) * l + frac * r).ToC4b()

    [<EntryPoint>]
    let main args =
        
        Aardvark.Init()

        let lf = @"/Users/atti/bla/DJI_Befliegung/Kopteraufnahmen/DJI_0753.JPG"
        let rf = @"/Users/atti/bla/DJI_Befliegung/Kopteraufnahmen/DJI_0754.JPG"

        let limg = (PixImage.Create lf).ToPixImage<byte>()
        let rimg = (PixImage.Create rf).ToPixImage<byte>()


        Log.startTimed "Akaze"
        let lftrs = OpenCV.detectFeatures OpenCV.DetectorMode.Akaze limg
        let rftrs = OpenCV.detectFeatures OpenCV.DetectorMode.Akaze rimg
        Log.stop()
        Log.line "L = %A" limg.Size
        Log.line "LFtrs = %d" lftrs.points.Length
        Log.line "R = %A" rimg.Size
        Log.line "RFtrs = %d" rftrs.points.Length

        Log.startTimed "kd"
        let lpts = lftrs.points |> Array.map (fun (f : KeyPoint) -> ((f.Position / V2d limg.Size) * 2.0 - V2d.II) * V2d.PN)
        let rpts = rftrs.points |> Array.map (fun (f : KeyPoint) -> ((f.Position / V2d rimg.Size) * 2.0 - V2d.II) * V2d.PN)

        let lkd = lpts.CreateRkdTreeDist2(1E-8)
        let rkd = rpts.CreateRkdTreeDist2(1E-8)
        Log.stop()

        let rec traverse (ct : int) (i : int) (bb : Box2d) (perm : int[]) (axis : int[]) (data : V2d[]) =
            if ct < 50 then 
                let fpp = float ct / bb.Area
                [fpp,bb]
            elif i < axis.Length then 
                let lct = (ct-1)/2
                let rct = ct-1-lct
                let ax = axis.[i]
                let sv = data.[perm.[i]].[ax]
                let mutable lb = bb
                lb.Max.[ax] <- sv
                let mutable rb = bb
                rb.Min.[ax] <- sv
                
                traverse lct (2*i+1) lb perm axis data @
                traverse rct (2*i+2) rb perm axis data
            else []

        let rec ftrTraverse (fppr : Range1d) (ct : int) (i : int) (bb : Box2d) (perm : int[]) (axis : int[]) (data : V2d[]) =
            if ct < 50 then 
                let fpp = float ct / bb.Area
                if fppr.Contains(fpp) then 
                    let rec traversePts i = 
                        if i < perm.Length then
                            let pt = data.[perm.[i]]
                            if i < axis.Length then
                               let li = ((i<<<1) ||| 1)
                               pt ::
                               traversePts li @  
                               traversePts (li+1)
                            else [pt]                           
                        else []                        
                    traversePts i
                else []        
            elif i < axis.Length then 
                let lct = (ct-1)/2
                let rct = ct-1-lct
                let ax = axis.[i]
                let sv = data.[perm.[i]].[ax]
                let mutable lb = bb
                lb.Max.[ax] <- sv
                let mutable rb = bb
                rb.Min.[ax] <- sv
                ftrTraverse fppr lct (2*i+1) lb perm axis data @
                ftrTraverse fppr rct (2*i+2) rb perm axis data
            else []


        Log.start "Traverse"
        let lfpps,lbs = traverse lpts.Length 0 (Box2d lpts) (Array.map int lkd.Data.PermArray) lkd.Data.AxisArray lpts |> List.unzip
        let rfpps,rbs = traverse rpts.Length 0 (Box2d rpts) (Array.map int rkd.Data.PermArray) rkd.Data.AxisArray rpts |> List.unzip

        let lmaxfpp = List.max lfpps
        let rmaxfpp = List.max rfpps
        let lfppr = Range1d(0.0 * lmaxfpp, 1.0 * lmaxfpp)
        let rfppr = Range1d(0.0 * rmaxfpp, 1.0 * rmaxfpp)
        let lfpppts = ftrTraverse lfppr lpts.Length 0 (Box2d lpts) (Array.map int lkd.Data.PermArray) lkd.Data.AxisArray lpts 
        let rfpppts = ftrTraverse rfppr rpts.Length 0 (Box2d rpts) (Array.map int rkd.Data.PermArray) rkd.Data.AxisArray rpts
        Log.stop()

        Log.line "Remaining lfpppts: %d" lfpppts.Length      
        Log.line "Remaining rfpppts: %d" rfpppts.Length      

        let rand = RandomSystem()
        let boxSg (bs : list<Box2d>) (fpps : list<float>) (maxfpp : float)=        
            (bs,fpps) ||> List.map2 (fun b fpp -> 
                let rel = fpp / maxfpp
                let col = heat rel
                Box3d(V3d(b.Min,-0.0001),V3d(b.Max,0.0001))
                    |> Sg.box' (C4b(col.R,col.G,col.B,100uy))
                    |> Sg.shader {
                        do! DefaultSurfaces.trafo
                        do! DefaultSurfaces.vertexColor
                    }
            )
            |> Sg.ofList
            |> Sg.blendMode (AVal.constant BlendMode.Blend)
            |> Sg.depthTest (AVal.constant DepthTestMode.None)
            |> Sg.pass (RenderPass.after "asdasd" RenderPassOrder.Arbitrary RenderPass.main)
            |> Sg.viewTrafo (AVal.constant Trafo3d.Identity)
            |> Sg.projTrafo (AVal.constant Trafo3d.Identity)

        let imgSg (img : PixImage) =
            Sg.fullScreenQuad
            |> Sg.diffuseTexture (AVal.constant (PixTexture2d(PixImageMipMap [| img |], TextureParams.mipmapped) :> ITexture))
            |> Sg.shader {
                do! DefaultSurfaces.diffuseTexture
            }

        let ftrSg (ftrs : V2d[]) =
            IndexedGeometryPrimitives.points (ftrs |> Array.map (fun v -> V3f(V2f v,-1.0f))) (Array.create ftrs.Length (C4b.Red))
            |> Sg.ofIndexedGeometry
            |> Sg.shader {
                do! DefaultSurfaces.trafo
                do! DefaultSurfaces.pointSprite
                do! DefaultSurfaces.pointSpriteFragment
                do! DefaultSurfaces.vertexColor
            }
            |> Sg.uniform "PointSize" (AVal.constant 4.0)
            |> Sg.depthTest (AVal.constant DepthTestMode.None)
            |> Sg.viewTrafo (AVal.constant Trafo3d.Identity)
            |> Sg.projTrafo (AVal.constant Trafo3d.Identity)
            |> Sg.pass (RenderPass.after "asdasd" RenderPassOrder.Arbitrary RenderPass.main)

        let win = 
            window {
                backend Backend.GL
            }

        let current = AVal.init 0
        win.Keyboard.DownWithRepeats.Values.Add ( fun k -> 
            match k with
            | Keys.Space -> transact(fun _ -> current.Value <- (current.Value + 1) % 2)
            | _ -> ()
        )

        let lsg =
            Sg.ofList [
                boxSg lbs lfpps lmaxfpp
                imgSg limg
                ftrSg (List.toArray lfpppts)
            ]   

        let rsg = 
            Sg.ofList [
                boxSg rbs rfpps rmaxfpp
                imgSg rimg
                ftrSg (List.toArray rfpppts)
            ]             

        let sg = 
            Sg.ofList [
                Sg.onOff (current |> AVal.map (fun v -> v=0)) lsg      
                Sg.onOff (current |> AVal.map (fun v -> v=1)) rsg           
            ]            

        win.Scene <- sg
        win.Run()

        0