namespace Aardvark.Reconstruction.Epipolar.Demo

open System
open Aardvark.Rendering
open Aardvark.Reconstruction
open Aardvark.Base
open FSharp.Data.Adaptive
open Aardvark.Rendering.Text
open Aardvark.SceneGraph
open Aardvark.Application
open Aardvark.Application.Utilities
open Aardvark.Application.Slim
open FShade
open FsCheck

[<AutoOpen>]
module ArbDemo = 
    open Lala

    let sg3d (s : ScenarioData) =
        let pts3d = 
            s.pts3d
            |> Array.map (fun p -> 
                let s = Sphere3d(p,0.05)
                IndexedGeometryPrimitives.solidSubdivisionSphere s 2 C4b.Red
                |> Sg.ofIndexedGeometry
            ) 
            |> Sg.ofArray
            |> Sg.shader {
                do! DefaultSurfaces.trafo
                do! DefaultSurfaces.vertexColor
                do! DefaultSurfaces.simpleLighting            
            }
        
        Sg.ofList [pts3d]

    let showArbScenario 
        (win : ISimpleRenderWindow)
        (scenario : aval<ScenarioData>) 
        (cf : aval<Option<Camera>>) 
        (ch : aval<Option<Camera>>) 
        (cp : aval<Option<Camera>>) =
        
        let c0 = scenario |> AVal.map (fun s -> s.cam0)
        let c1 = scenario |> AVal.map (fun s -> s.cam1)
        Log.startTimed("Show")

        let vpo = AVal.map2 (fun (v : Trafo3d[]) (p : Trafo3d[]) -> v.[0] * p.[0]) win.View win.Proj
        let vp0 = c0 |> AVal.map(Camera.viewProjTrafo 0.1 100.0)
        let vp1 = c1 |> AVal.map(Camera.viewProjTrafo 0.1 100.0)
        let vph = ch |> AVal.map(Option.map(Camera.viewProjTrafo 0.1 100.0))
        let vpf = cf |> AVal.map(Option.map(Camera.viewProjTrafo 0.1 100.0))
        let vpp = cp |> AVal.map(Option.map(Camera.viewProjTrafo 0.1 100.0))

        let current = AVal.init 0
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
            | _ -> ()
        )

        let vt = current |> AVal.bind (fun c -> 
            match c with
            | 0 -> vpo
            | 1 -> vp0
            | 2 -> vp1
            | 3 -> AVal.map2 (fun o t -> t |> Option.defaultValue o) vpo vph
            | 4 -> AVal.map2 (fun o t -> t |> Option.defaultValue o) vpo vpf
            | 5 -> AVal.map2 (fun o t -> t |> Option.defaultValue o) vpo vpp
            | _ -> vpo
        )

        let statusText =
            let t = current |> AVal.map (fun c -> 
                match c with 
                | 0 -> "Freefly"
                | 1 -> "LeftCam"
                | 2 -> "True RightCam"
                | 3 -> "Homography"
                | 4 -> "Fundamental"
                | 5 -> "P6P"
                | _ -> "?"
            )
            Sg.text FontSquirrel.HamburgerHeaven.Regular C4b.White t
            |> Sg.scale 0.1
            |> Sg.translate -0.9 -0.9 0.0
            |> Sg.viewTrafo (AVal.constant Trafo3d.Identity)
            |> Sg.projTrafo (AVal.constant Trafo3d.Identity)

        let sg3d = 
            scenario
            |> AVal.map sg3d
            |> Sg.dynamic
            |> Sg.viewTrafo vt
            |> Sg.projTrafo (AVal.constant Trafo3d.Identity)

        let ftrSg = 
            AVal.custom (fun t -> 
                let current = current.GetValue(t)
                let s = scenario.GetValue(t)

                let ls = s.matches |> Array.map fst
                let rs = s.matches |> Array.map snd
                let obs = 
                    match current with 
                    | 1 -> ls
                    | 2 | 3 | 4 | 5 -> rs
                    | _ -> [||]
                match obs with 
                | [||] -> Sg.empty 
                | _ -> 
                    let s1 = 
                        let ps = obs |> Array.map (fun p -> V3d(p.X,p.Y,-1.0))
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
                          |> Sg.uniform "PointSize" (AVal.constant 3.5)
                          |> Sg.depthTest (AVal.constant DepthTest.None)
                    let s2 = 
                        let ps = obs |> Array.map (fun p -> V3d(p.X,p.Y,-1.0))
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
                          |> Sg.uniform "PointSize" (AVal.constant 2.0)
                          |> Sg.depthTest (AVal.constant DepthTest.None)
                          |> Sg.pass (RenderPass.after "asd" RenderPassOrder.Arbitrary RenderPass.main)
                    Sg.ofList [s1;s2]
            ) |> Sg.dynamic

        let frustSg =
            AVal.custom (fun t -> 
                let current = current.GetValue(t)
                let c0 = c0.GetValue(t)
                let c1 = c1.GetValue(t)
                let ch = ch.GetValue(t)
                let cf = cf.GetValue(t)
                let cp = cp.GetValue(t)
                if current = 0 then
                    [
                        yield Aardvark.Reconstruction.Epipolar.Demo.Sg.camera (AVal.constant 0.49) c0 C4b.Blue
                        yield Aardvark.Reconstruction.Epipolar.Demo.Sg.camera (AVal.constant 0.49) c1 C4b.Blue
                        yield! cf |> Option.map(fun c -> Aardvark.Reconstruction.Epipolar.Demo.Sg.camera (AVal.constant 0.50) c C4b.Green             )   |> Option.toList
                        yield! ch |> Option.map(fun c -> Aardvark.Reconstruction.Epipolar.Demo.Sg.camera (AVal.constant 0.51) c C4b.Red               )   |> Option.toList
                        yield! cp |> Option.map(fun c -> Aardvark.Reconstruction.Epipolar.Demo.Sg.camera (AVal.constant 0.52) c (C4b(100,255,255,255)))   |> Option.toList
                    ] |> Sg.ofList
                else Sg.empty
            ) |> Sg.dynamic

        let outlineSg =
            scenario |> AVal.map (fun s -> 
                match s.points with
                | AlmostLinearQuad(q,f) -> 
                    let q = flattenQuad q f
                    IndexedGeometryPrimitives.quad' q.P0 q.P1 q.P2 q.P3 C4b.Yellow    
                | AlmostFlatVolume(b,f) -> 
                    let b = flattenBox b f
                    IndexedGeometryPrimitives.Box.wireBox b C4b.Yellow
                | AlmostLinearVolume(b,f) -> 
                    let b = linearizeBox b f
                    IndexedGeometryPrimitives.Box.wireBox b C4b.Yellow
                | InQuad q -> 
                    IndexedGeometryPrimitives.quad' q.P0 q.P1 q.P2 q.P3 C4b.Red    
                | InVolume b -> 
                    IndexedGeometryPrimitives.Box.wireBox b C4b.Red            
            )
            |> AVal.map Sg.ofIndexedGeometry
            |> Sg.dynamic
            |> Sg.shader {
                do! DefaultSurfaces.trafo
                do! DefaultSurfaces.thickLine
                do! DefaultSurfaces.vertexColor
            }
            |> Sg.uniform "LineWidth" (AVal.constant 1.0)
            |> Sg.viewTrafo vt
            |> Sg.projTrafo (AVal.constant Trafo3d.Identity)

        let sg = 
            Sg.ofList [
                frustSg        
                ftrSg
                sg3d
                statusText
                outlineSg
            ]

        win.Scene <- sg
        Log.stop()
        win.Run()


    let showScenario 
        (win : ISimpleRenderWindow)
        (scenario : aval<ScenarioData>) 
        (co : aval<Option<Camera>>)
        (name : aval<string>) =
        
        let c0 = scenario |> AVal.map (fun s -> s.cam0)
        let c1 = scenario |> AVal.map (fun s -> s.cam1)
        Log.startTimed("Show")

        let vpo = AVal.map2 (fun (v : Trafo3d[]) (p : Trafo3d[]) -> v.[0] * p.[0]) win.View win.Proj
        let vp0 = c0 |> AVal.map(Camera.viewProjTrafo 0.1 100.0)
        let vp1 = c1 |> AVal.map(Camera.viewProjTrafo 0.1 100.0)
        let vpot = co |> AVal.map(Option.map(Camera.viewProjTrafo 0.1 100.0))

        let current = AVal.init 0
        win.Keyboard.DownWithRepeats.Values.Add (fun k -> 
            match k with
            | Keys.D1 -> 
                transact (fun _ -> current.Value <- 0)
            | Keys.D2 -> 
                transact (fun _ -> current.Value <- 1)
            | Keys.D3 -> 
                transact (fun _ -> current.Value <- 2)
            | Keys.D4 -> 
                transact (fun _ -> current.Value <- 3)
            | Keys.Space -> 
                transact (fun _ -> current.Value <- (current.Value + 1)%4)
            | _ -> ()

            // match current.Value with 
            // | 0 -> Log.line "freefly"
            // | 1 -> Log.line "Camera 0 original"
            // | 2 -> Log.line "Camera 1 original"
            // | 3 -> Log.line "Camera 1 %s" (name |> AVal.force)
            // | _ -> ()
        )

        let vt = current |> AVal.bind (fun c -> 
            match c with
            | 0 -> vpo
            | 1 -> vp0
            | 2 -> vp1
            | 3 -> AVal.map2 (fun o t -> t |> Option.defaultValue o) vpo vpot
            | _ -> vpo
        )

        let statusText =
            let t = 
                AVal.map2 (fun c n -> 
                    match c with 
                    | 0 -> "Freefly"
                    | 1 -> "LeftCam"
                    | 2 -> "True RightCam"
                    | 3 -> n
                    | _ -> "?"
                ) current name
            Sg.text FontSquirrel.HamburgerHeaven.Regular C4b.White t
            |> Sg.scale 0.1
            |> Sg.translate -0.9 -0.9 0.0
            |> Sg.viewTrafo ~~Trafo3d.Identity
            |> Sg.projTrafo ~~Trafo3d.Identity

        let sg3d = 
            scenario
            |> AVal.map sg3d
            |> Sg.dynamic
            |> Sg.viewTrafo vt
            |> Sg.projTrafo (AVal.constant Trafo3d.Identity)

        let ftrSg = 
            AVal.custom (fun t -> 
                let current = current.GetValue(t)
                let s = scenario.GetValue(t)

                let ls = s.matches |> Array.map fst
                let rs = s.matches |> Array.map snd
                let obs = 
                    match current with 
                    | 1 -> ls
                    | 2 | 3 -> rs
                    | _ -> [||]
                match obs with 
                | [||] -> Sg.empty 
                | _ -> 
                    let s1 = 
                        let ps = obs |> Array.map (fun p -> V3d(p.X,p.Y,-1.0))
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
                          |> Sg.uniform "PointSize" (AVal.constant 3.5)
                          |> Sg.depthTest (AVal.constant DepthTest.None)
                    let s2 = 
                        let ps = obs |> Array.map (fun p -> V3d(p.X,p.Y,-1.0))
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
                          |> Sg.uniform "PointSize" (AVal.constant 2.0)
                          |> Sg.depthTest (AVal.constant DepthTest.None)
                          |> Sg.pass (RenderPass.after "asd" RenderPassOrder.Arbitrary RenderPass.main)
                    Sg.ofList [s1;s2]
            ) |> Sg.dynamic

        let frustSg =
            AVal.custom (fun t -> 
                let current = current.GetValue(t)
                let c0 = c0.GetValue(t)
                let c1 = c1.GetValue(t)
                let co = co.GetValue(t)
                if current = 0 then
                    [
                        yield Aardvark.Reconstruction.Epipolar.Demo.Sg.camera ~~0.49 c0 C4b.Blue
                        yield Aardvark.Reconstruction.Epipolar.Demo.Sg.camera ~~0.49 c1 C4b.Blue
                        yield! co |> Option.map(fun c -> Aardvark.Reconstruction.Epipolar.Demo.Sg.camera ~~0.51 c C4b.Red               )   |> Option.toList
                    ] |> Sg.ofList
                else Sg.empty
            ) |> Sg.dynamic

        let outlineSg =
            scenario |> AVal.map (fun s -> 
                match s.points with
                | AlmostLinearQuad(q,f) -> 
                    let q = flattenQuad q f
                    IndexedGeometryPrimitives.quad' q.P0 q.P1 q.P2 q.P3 C4b.Yellow    
                | AlmostFlatVolume(b,f) -> 
                    let b = flattenBox b f
                    IndexedGeometryPrimitives.Box.wireBox b C4b.Yellow
                | AlmostLinearVolume(b,f) -> 
                    let b = linearizeBox b f
                    IndexedGeometryPrimitives.Box.wireBox b (C4b(255,255,100,255))
                | InQuad q -> 
                    IndexedGeometryPrimitives.quad' q.P0 q.P1 q.P2 q.P3 C4b.Red    
                | InVolume b -> 
                    IndexedGeometryPrimitives.Box.wireBox b C4b.Red            
            )
            |> AVal.map Sg.ofIndexedGeometry
            |> Sg.dynamic
            |> Sg.shader {
                do! DefaultSurfaces.trafo
                do! DefaultSurfaces.thickLine
                do! DefaultSurfaces.vertexColor
            }
            |> Sg.uniform "LineWidth" ~~1.0
            |> Sg.viewTrafo vt
            |> Sg.projTrafo (AVal.constant Trafo3d.Identity)

        let sg = 
            Sg.ofList [
                frustSg        
                ftrSg
                sg3d
                statusText
                outlineSg
            ]

        win.Scene <- sg
        Log.stop()
        win.Run()

    let singleArbTest() =
        Aardvark.Init()
        let win = window {
            backend Backend.GL
        }
        let rand = RandomSystem()

        let counter = AVal.init 0
        win.Keyboard.DownWithRepeats.Values.Add (function 
            | Keys.Enter -> 
                transact (fun _ -> counter.Value <- counter.Value + 1)
            | _ -> ()
        )
        
        let pickler = MBrace.FsPickler.FsPickler.CreateBinarySerializer()

        let stuff =
            counter |> AVal.map ( fun _ -> 
                Log.startTimed("Generate Scenario")

                //let scenario = Gen.eval 0 (Random.StdGen(rand.UniformInt(),rand.UniformInt())) Lala.genPlaneScenario 
                let scenario : Scenario = @"/Users/atti/temp/dump.bin" |> File.readAllBytes |> pickler.UnPickle

                Log.line "Scenario:\n"
                //Log.line "C0:%A" scene.cam0
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
                                Log.warn "No fundamental possible"
                                []
                            | Some (fund,lsbr,rsbl) -> 
                                Log.line "lsbr %A" lsbr
                                Log.line "rsbl %A" rsbl
                                match FundamentalMatrix.decompose fund c0.proj c1.proj [] with
                                | [] -> 
                                    Log.warn "Fundamental decompose failed"
                                    []
                                | fs -> 
                                    Log.line "Decomposed Fundamental into %d motions" fs.Length
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
                                Log.warn "No homography possible"
                                []
                            | Some h ->                                
                                match Homography.decompose h c0.proj c1.proj [] with
                                | [] -> 
                                    Log.warn "Homography decompose failed"
                                    []
                                | hs -> 
                                    Log.line "Decomposed Homography into %d motions" hs.Length
                                    hs |> List.map (fun m -> m * scale)
                        name,recover,scene

                let pMatches =
                    Array.zip scene.pts3d scene.matches
                    |> Array.map (fun (p3d,(l,r)) -> r,p3d)
                Log.stop()

                Log.startTimed("Calculate epipolar")

                let mots = recover()
                let co = getBestFittingMot scene.cam0 scene.cam1 scene.pts3d scene.matches mots |> Option.map (fun m -> { (scene.cam0 + m) with proj = scene.cam1.proj })

                let fu c n = 
                    match c with
                    | None -> Log.warn "No camera: %s" n
                    | Some c -> 
                        let d = 
                            pMatches |> Array.averageBy (fun (o,p) -> 
                                let ndc = Camera.projectUnsafe p c
                                let d = Vec.distance o ndc
                                d
                            )
                        if d > 1E-3 then Log.error "bad cam: %s (%f)" n d
                        Log.line "AvgNdcErr: %.9f" d
                        Log.line "sameness %s %f" n (Camera.sameness scene.cam1 c)

                fu co name

                Log.line "Kind: %s" name
                Log.line "C1Trans:%A" scene.camtrans
                Log.line "C1Rot:%A" scene.camrot
                Log.line "pts3d:%A" scene.points
                Log.line "noise:%A" scene.noise

                Log.stop()
                scene, co, name
            )
    
        let scene = AVal.map       (fun (x,_,_) -> x) stuff
        let co = AVal.map          (fun (_,x,_) -> x) stuff
        let name = AVal.map        (fun (_,_,x) -> x) stuff
        
        showScenario win scene co name

    let rand = RandomSystem()

    let manyArbsAndRenderIfBad() =
        let mutable bad = None
        let tryRun() =
            let scenario = Gen.eval 0 (Random.StdGen(rand.UniformInt(),rand.UniformInt())) Lala.genPlaneScenario 
            let scene = match scenario with FundamentalScenario scene -> scene | HomographyScenario scene -> scene
            let c0 = scene.cam0
            let c1 = scene.cam1
            let pMatches =
                Array.zip scene.pts3d scene.matches
                |> Array.map (fun (p3d,(l,r)) -> r,p3d)
            let scale = (c1.Location - c0.Location).Length

            let matches = scene.matches
            let hom = Homography.recover matches
            let hmot =
                match hom with
                | None -> 
                    []
                | Some h -> 
                    match Homography.decompose h c0.proj c1.proj [] with
                    | [] -> 
                        []
                    | hs -> 
                        hs |> List.map (fun m -> m * scale)
            
            let fmot =
                let f = FundamentalMatrix.recover 1E-4 matches
                match f with
                | None -> 
                    []
                | Some (fund,lsbr,rsbl) -> 
                    match FundamentalMatrix.decompose fund c0.proj c1.proj [] with
                    | [] -> 
                        []
                    | fs -> 
                        fs |> List.map (fun m -> m * scale)

            let cp : Option<Camera> = None
                // match P6P.recover pMatches with
                // | None -> 
                //     None
                // | Some cp -> 
                //     Some cp

            let cf = getBestFittingMot c0 c1 scene.pts3d scene.matches fmot |> Option.map (fun m -> { (c0 + m) with proj = c1.proj })
            let ch = getBestFittingMot c0 c1 scene.pts3d scene.matches hmot |> Option.map (fun m -> { (c0 + m) with proj = c1.proj })

            let mutable testy = 0
            let fu c n = 
                match c with
                | None -> ()
                | Some c -> 
                    let d = 
                        pMatches |> Array.averageBy (fun (o,p) -> 
                            let ndc = Camera.projectUnsafe p c
                            let d = Vec.distance o ndc
                            d
                        )

                    if d > 1E-3 then testy <- testy+1

            fu ch "Homography"
            fu cf "Fundamental"
            //fu cp "P6P"

            let pickler = MBrace.FsPickler.FsPickler.CreateBinarySerializer()

            if testy >= 2 then 
                Log.error "bad found:%A" scene
                pickler.Pickle(scenario) |> File.writeAllBytes @"/Users/atti/temp/dump.bin"
                bad <- Some (scene, cf, ch, cp)

        let mutable ct = 0
        let max = 100000
        Log.startTimed "Running baddies"
        while bad |> Option.isNone && ct < max do
            Report.Progress(float ct/float max)
            tryRun()
            ct <- ct+1
        Report.Progress(1.0)
        Log.stop()        
        if bad |> Option.isNone then Log.line "juhu"
        else
            Aardvark.Init()
            let win = window {
                backend Backend.GL
            }

            Log.error "bad found:%A" ct
            let (scene, cf, ch, cp) = bad |> Option.get

            showArbScenario win ~~scene ~~cf ~~ch ~~cp