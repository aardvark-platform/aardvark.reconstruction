namespace Aardvark.Reconstruction.Epipolar.Demo 

open Aardvark.Base
open Aardvark.Base.Incremental
open Aardvark.Base.Incremental.Operators
open Aardvark.Reconstruction
open Aardvark.Base.Rendering
open Aardvark.SceneGraph
open FsCheck

module IGP =
    let solidQuadrangleOfPlane ((n,p) : V3d*V3d) (scale : float) (color : C4b) =
        let d1 = Vec.cross n V3d.OOI
        let d2 = Vec.cross d1 n
        let p0 = p + scale * d1
        let p1 = p + scale * d2
        let p2 = p - scale * d1
        let p3 = p - scale * d2

        IndexedGeometryPrimitives.Quad.solidQuadrangleWithColorsAndNormals 
            (p0,color,n) 
            (p1,color,n)  
            (p2,color,n)  
            (p3,color,n)  

module Sg =
    let unitFrustum col =
        let call = DrawCallInfo(FaceVertexCount = 16, InstanceCount = 1)

        let positions =
            [|
                V3f.Zero
                V3f(-1.0f, -1.0f, -1.0f)
                V3f( 1.0f, -1.0f, -1.0f)
                V3f( 1.0f,  1.0f, -1.0f)
                V3f(-1.0f,  1.0f, -1.0f)
            |]

        let colors =
            [|
                col
                col
                col
                col
                col
            |]

        let index =
            [|
                0; 1
                0; 2
                0; 3
                0; 4
                1; 2
                2; 3
                3; 4
                4; 1
            |]

        Sg.render IndexedGeometryMode.LineList call
            |> Sg.vertexAttribute DefaultSemantic.Positions ~~positions
            |> Sg.vertexAttribute DefaultSemantic.Colors ~~colors
            |> Sg.index ~~index
    let camera (scale : IMod<float>) (c : Camera) (col : C4b) =
        let view = Camera.viewTrafo c
        
        let center = Sg.lines ~~C4b.Red ~~[| Line3d(V3d.Zero, -V3d.OOI) |]


        let pp = -c.proj.distortion.principalPoint
        

        Sg.ofList [ 
            unitFrustum col
            |> Sg.transform (Trafo3d.ShearXY(pp.X, pp.Y) * Trafo3d.Scale(1.0, 1.0 / c.proj.aspect, c.proj.focalLength))
               
            center
            |> Sg.transform (Trafo3d.Scale(1.0, 1.0 / c.proj.aspect, c.proj.focalLength))
        ]
        |> Sg.trafo (scale |> Mod.map Trafo3d.Scale)
        |> Sg.transform view.Inverse
        |> Sg.shader {
            do! DefaultSurfaces.trafo
            do! DefaultSurfaces.thickLine
            do! DefaultSurfaces.thickLineRoundCaps
            //do! DefaultSurfaces.vertexColor
        }
        |> Sg.uniform "LineWidth" ~~3.0

module Generate = 
    let rand = RandomSystem()
    let mkSg points3d =
        points3d |> List.collect (fun (n,p,pts) -> 
            let pcol = rand.UniformC3f().ToC4b()
            let fcol = rand.UniformC3f().ToC4b()
            [
                yield IGP.solidQuadrangleOfPlane (n,p) 1.0 pcol
                yield! pts |> List.map (fun p -> 
                    IndexedGeometryPrimitives.Sphere.solidSubdivisionSphere (Sphere3d(p,0.025)) 2 fcol
                )
            ]
        ) |> List.map Sg.ofIndexedGeometry
            |> Sg.ofList
            |> Sg.shader {
            do! DefaultSurfaces.trafo
            do! DefaultSurfaces.vertexColor
            do! DefaultSurfaces.simpleLighting
            }

    let rec randCond (create : RandomSystem -> 'a) (check : 'a -> bool) =
        let mutable tries = 0
        Seq.initInfinite (fun _ -> 
            tries <- tries + 1
            if tries < 10000 then
                create rand
            else
                Log.warn "bad rand" 
                create rand
                
        ) |> Seq.filter check
          |> Seq.head

    let inline floatBetween (min : float) (max : float) = min + rand.UniformDouble() * (max - min)
    let inline intBetween (min : int) (max : int) = min + rand.UniformInt(max - min)
    
    let randomProjection() =
        let aspect = floatBetween 1.0 2.370
        let focal = floatBetween 0.67 2.0
        let imageSize = 
            let x = intBetween 640 3840
            let y = intBetween 480 2160
            V2i(x,y)
        let pp = rand.UniformV2d(Box2d(V2d(-0.1,-0.1),V2d(0.1,0.1)))
        
        { Projection.identity with 
            aspect = aspect 
            focalLength = focal
            distortion = { imageSize = imageSize; distortion = RadialDistortion2d.Identity; principalPoint = pp }
        }
        
    let randomScene() =
        let scale = 4.0
        let p0 = V3d.IOO * 2.0 * scale
        let cv0 = CameraView.lookAt p0 V3d.OOO V3d.OOI
        let trans = 
            let dir = 
                let xy = rand.UniformV2d(Box2d(V2d.NN,V2d.II))
                let off = randCond (fun rand -> scale * 0.25 * rand.UniformDouble()) (fun v -> abs v > 0.5)
                V3d(off, xy.X, xy.Y)
            let len = randCond (fun rand -> (rand.UniformDouble() * 2.0 - 1.0)*scale) (fun v -> abs v > 1.5)
            dir * len
        let p1 = p0 + trans        
        let cv1 = 
            let target = rand.UniformV3d(Box3d(V3d.NNN, V3d.III))
            CameraView.lookAt p1 target V3d.OOI
        let c0 = { view = cv0; proj = randomProjection() }
        let c1 = { view = cv1; proj = randomProjection() }

        let HQuads, FQuads =
            let num = rand.UniformInt(3) + 2
            let basequads = 
                List.init num (fun _ -> 
                    let p = 
                        randCond 
                            (fun rand -> 
                                let v2 = rand.UniformV2d(Box2d(V2d.NN * scale * 0.5,V2d.II * scale * 0.5))
                                V3d(0.0, v2.X, v2.Y)    
                            )
                            (fun v -> v.Length > scale / 8.0)
                    let n = 
                        let krassness = scale * 1.0
                        let n = 
                            let target = 
                                randCond (fun rand -> rand.UniformV3d(Box3d(V3d(p1.X, p1.Y-krassness, p1.Z-krassness),V3d(p1.X, p1.Y+krassness, p1.Z+krassness)))
                                ) (fun t -> (p1-t).Length > 0.1)
                            (target - p).Normalized    
                        n
                        
                    2,n,p
                )
            let HQuads = 
                basequads |> List.take 1

            let FQuads =
                let (_,pno,pp) = basequads |> List.last 
                let pn =  (Rot3d(V3d.III.Normalized,45.0*Constant.RadiansPerDegree).TransformPos pno).Normalized
                let n1 = Vec.cross pn V3d.OOI
                let n2 = Vec.cross n1 pn
                let pp2 = pp + V3d.OII * scale * 0.25
                let pn2 = (Rot3d(V3d.PNN.Normalized,60.0*Constant.RadiansPerDegree).TransformPos pno).Normalized
                let n12 = Vec.cross pn2 V3d.OOI
                let n22 = Vec.cross n12 pn2
                let pp3 = pp2 + V3d.PON * scale * 0.25
                let pn3 = (Rot3d(V3d.OOI.Normalized,90.0*Constant.RadiansPerDegree).TransformPos pn2).Normalized
                let n13 = Vec.cross pn3 V3d.OOI
                let n23 = Vec.cross n13 pn3
                [   1,pn, pp
                    1,n1, pp
                    1,n2, pp
                    2,pn2,pp2
                    2,n12,pp2
                    2,n22,pp2
                    1,pn3,pp3
                    1,n13,pp3
                    1,n23,pp3
                ]
            HQuads, FQuads

        let points3d quads =
            quads |> List.map (fun (w,n,p) -> 
                let n = n |> Vec.normalize
                let dy = Vec.cross n V3d.OOI |> Vec.normalize
                let dx = Vec.cross dy n      |> Vec.normalize
                let t = Euclidean3d(Rot3d.FromM33d(M33d.FromCols(dx,dy,n)), p)

                let inline rnd (n : int) (b : Box3d) =
                    List.init n (fun _ -> 
                        rand.UniformV3d(b.Transformed(Trafo3d.Scale(0.1 * scale)))
                        |> t.TransformPos
                    )

                let pts = 
                    List.concat [
                        rnd w (Box3d(V3d.NNO,V3d.OOO))
                        rnd w (Box3d(V3d.ONO,V3d.IOO))
                        rnd w (Box3d(V3d.NOO,V3d.OIO))
                        rnd w (Box3d(V3d.OOO,V3d.IIO))
                    ]
                    
                n,p, pts
            )

        let Hpoints3d = points3d HQuads
        let Fpoints3d = points3d FQuads

        let Hpoints2dc0 =
            Hpoints3d |> List.map (fun (n,p,pts) -> 
                n,p, pts |> List.map (fun pt -> Camera.projectUnsafe pt c0)
            )
        let Hpoints2dc1 =
            Hpoints3d |> List.map (fun (n,p,pts) -> 
                n,p, pts |> List.map (fun pt -> Camera.projectUnsafe pt c1)
            )
        let Fpoints2dc0 =
            Fpoints3d |> List.map (fun (n,p,pts) -> 
                n,p, pts |> List.map (fun pt -> Camera.projectUnsafe pt c0)
            )
        let Fpoints2dc1 =
            Fpoints3d |> List.map (fun (n,p,pts) -> 
                n,p, pts |> List.map (fun pt -> Camera.projectUnsafe pt c1)
            )
            
        c0, c1, Hpoints2dc0, Hpoints2dc1, Fpoints2dc0, Fpoints2dc1, Hpoints3d, Fpoints3d

module Lala =
    //let rand = RandomSystem()

    let reasonableFloat =
        Arb.generate<NormalFloat> 
        |> Gen.filter (fun (NormalFloat v) -> not (abs v > 1E10))
        |> Gen.map (fun (NormalFloat v) -> v)

    let reasonableInt =
        Gen.choose(-10000,10000)

    let reasonableAngleRad =
        reasonableFloat
        |> Gen.map (fun v -> v % Constant.PiTimesTwo)

    let floatBetween min max =
        let size = max - min
        Arb.generate<uint32>
        |> Gen.map (fun i -> (float i / float System.UInt32.MaxValue) * size + min)
        
    let arbV2d (b : Box2d) =
        gen {
            let! x = floatBetween b.Min.X b.Max.X
            let! y = floatBetween b.Min.Y b.Max.Y
            return V2d(x,y)
        }    

    let arbV3d (b : Box3d) =
        gen {
            let! x = floatBetween b.Min.X b.Max.X
            let! y = floatBetween b.Min.Y b.Max.Y
            let! z = floatBetween b.Min.Z b.Max.Z
            return V3d(x,y,z)
        }        

    let arbPos =
        gen {
            let! x = reasonableFloat
            let! y = reasonableFloat
            let! z = reasonableFloat
            return V3d(x,y,z)
        }
    
    let arbDir =
        gen {
            let! phi = reasonableAngleRad
            let! z = floatBetween -1.0 1.0
            let s = sqrt (1.0 - z * z)
            return V3d(cos phi * s, sin phi * s, z)
        }    

    let arbProjection =
        gen {
            let! aspect = floatBetween 0.7 2.6
            let! focal = floatBetween 0.5 3.0
            let! px = Gen.choose(320,7680)
            let! py = Gen.choose(240,4320)
            let sizes = V2i(px,py)
            let! pp = arbV2d (Box2d(V2d(-0.5,-0.5),V2d(0.5,0.5)))
            return 
                { Projection.identity with 
                    aspect = aspect 
                    focalLength = focal
                    distortion = { imageSize = sizes; distortion = RadialDistortion2d.Identity; principalPoint = pp }
                }
        }

    type ArbCameraTrans = 
        | None
        | AlongFw of float
        | InPlane of V2d
        | ArbitraryDir of V3d

    type RotModifier =
        | None
        | Roll of float
        | Normal
        | NormalAndRoll of float

    type ArbPoints =
        | AlongLine of Line3d
        | InQuad of Quad3d
        | InVolume of Box3d

    type NoiseKind =
        | Offset
        | Garbage
        | Mismatch

    let genRotModifier =
        gen {
            let! i = Gen.choose(0,3)
            match i with
            | 0 -> return RotModifier.None
            | 1 -> 
                let! a = reasonableAngleRad
                return Roll a
            | 2 -> return Normal
            | 3 -> 
                let! a = reasonableAngleRad
                return NormalAndRoll a
            | _ -> 
                return failwith "no"
        }

    let genCameraTrans (scale : float) =
        gen {
            let! i = Gen.choose(0,3)
            match i with
            | 0 -> return ArbCameraTrans.None
            | 1 -> 
                let sh = scale/2.0
                let! d = floatBetween -sh sh
                return AlongFw d
            | 2 -> 
                let sh = scale/2.0           
                let! dx = floatBetween -sh sh 
                let! dy = floatBetween -sh sh 
                return InPlane (V2d(dx,dy))
            | 3 -> 
                let sh = scale/2.0
                let! dir = arbDir
                let! len = floatBetween -sh sh
                return ArbitraryDir (dir*len)
            | _ -> 
                return failwith "no"
        }

    let applyTrans (c0 : Camera) (t : ArbCameraTrans) =
        match t with
        | ArbCameraTrans.None -> c0
        | AlongFw len -> 
            let p1 = c0.Location + c0.Forward * len
            { c0 with view = CameraView.lookAt p1 (p1 + c0.Forward) c0.Up }
        | InPlane d -> 
            let p1 = c0.Location + c0.Right * d.X + c0.Up * d.Y
            { c0 with view = CameraView.lookAt p1 (p1 + c0.Forward) c0.Up }
        | ArbitraryDir v -> 
            let p1 = c0.Location + v
            { c0 with view = CameraView.lookAt p1 (p1 + c0.Forward) c0.Up }

    // let applyRot (c0 : Camera) (dataBb : Box3d) (r : RotModifier) =
    //     match r with
    //     | None -> c0
    //     | Normal -> 

    // let generate() =
    //     gen {
    //         let! p0 = reasonablePos
    //         let! t0 = reasonablePos
    //         let u0 =  Vec.cross (Vec.cross p0 V3d.OOI) t0
    //         let cv0 = CameraView.lookAt p0 t0 u0
    //         let proj0 = arbProjection
    //         let cam0 = { view = cv0; proj = proj0 }
            
    //     }

    // let generate() =
    //     let 
