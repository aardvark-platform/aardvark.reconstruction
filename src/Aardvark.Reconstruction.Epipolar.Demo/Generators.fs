namespace Aardvark.Reconstruction.Epipolar.Demo 

open Aardvark.Base
open FSharp.Data.Adaptive
open Aardvark.Reconstruction
open Aardvark.Base.Rendering
open Aardvark.SceneGraph
open FsCheck

[<AutoOpen>]
module Util = 
    let inline (~~) (x : 'a) = AVal.constant x


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
    let camera (scale : aval<float>) (c : Camera) (col : C4b) =
        let view = Camera.viewTrafo c
        
        let center = Sg.lines ~~C4b.Red ~~[| Line3d(V3d.Zero, -V3d.OOI) |]


        let pp = -c.proj.distortion.principalPoint
        

        Sg.ofList [ 
            unitFrustum col
            |> Sg.transform (Trafo3d.ShearXY(pp.X, pp.Y) * Trafo3d.Scale(1.0, 1.0 / c.proj.aspect, c.proj.focalLength))
               
            center
            |> Sg.transform (Trafo3d.Scale(1.0, 1.0 / c.proj.aspect, c.proj.focalLength))
        ]
        |> Sg.trafo (scale |> AVal.map Trafo3d.Scale)
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
                let pn =  (Rot3d(45.0*Constant.RadiansPerDegree, V3d.III.Normalized).Transform pno).Normalized
                let n1 = Vec.cross pn V3d.OOI
                let n2 = Vec.cross n1 pn
                let pp2 = pp + V3d.OII * scale * 0.25
                let pn2 = (Rot3d(60.0*Constant.RadiansPerDegree, V3d.PNN.Normalized).Transform pno).Normalized
                let n12 = Vec.cross pn2 V3d.OOI
                let n22 = Vec.cross n12 pn2
                let pp3 = pp2 + V3d.PON * scale * 0.25
                let pn3 = (Rot3d(90.0*Constant.RadiansPerDegree, V3d.OOI.Normalized).Transform pn2).Normalized
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

[<AutoOpen>]
module Lala =
    let rand = RandomSystem()

    let floatBetween min max =
        let size = max - min
        gen { return rand.UniformDouble() * size + min }
        // //Gen.choose (0, System.Int32.MaxValue)
        // gen {
        //     return rand.UniformInt()            
        // }
        // |> Gen.map (fun i -> (float (abs i) / float System.Int32.MaxValue) * size + min)

    let intBetween min max =
        let range = max - min
        gen { return rand.UniformInt(range+1) + min }

    let reasonableFloat = floatBetween -10000.0 10000.0

    let reasonableAngleRad =
        reasonableFloat
        |> Gen.map (fun v -> v % Constant.PiTimesTwo)

        
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
            let! aspect = floatBetween 0.9 2.3
            let! focal = floatBetween 0.8 2.4
            let! px = Gen.choose(320,7680)
            let! py = Gen.choose(240,4320)
            let sizes = V2i(px,py)
            let! pp = arbV2d (Box2d(V2d(-0.05,-0.05),V2d(0.05,0.05)))
            return 
                { Projection.identity with 
                    aspect = aspect 
                    focalLength = focal
                    distortion = { imageSize = sizes; distortion = RadialDistortion2d.Identity; principalPoint = pp }
                }
        }

    type ArbCameraTrans = 
        | No
        | AlongFw of float
        | InPlane of V2d
        | ArbitraryDir of V3d

    type RotModifier =
        | Not
        | Normal
        | NormalAndRoll of float

    type ArbPoints =
        | AlmostLinearQuad of Quad3d * float
        | InQuad of Quad3d
        | AlmostFlatVolume of Box3d * float
        | AlmostLinearVolume of Box3d * float
        | InVolume of Box3d

    type NoiseKind =
        | Nope
        | Offset of sigma : float
        | Garbage of chance:float
        | OffsetAndGarbage of oChance:float * gChance:float

    let genNoiseKind =
        gen {
            let! i = Gen.choose(1,3)
            let! garbChance = floatBetween 0.0001 0.25
            let! sigma = floatBetween 0.000001 0.025
            //return Garbage garbChance
            //return Offset offsetProb
            //return Nope
            match i with
            | 0 -> return Nope
            | 1 -> return Offset sigma
            | 2 -> return Garbage garbChance
            | 3 -> return OffsetAndGarbage (sigma, garbChance)
            | _ -> return failwith "no"
        }

    
    let gauss = RandomGaussian(rand)
    let nextGaussian sigma =
        gen { return gauss.GetDouble(0.0,sigma) |> abs |> clamp 0.0 1.0 }

    let applyNoise (kind : NoiseKind) (ms : (V2d*V2d)[]) =
        gen {
            let! fs = Gen.arrayOfLength ms.Length (floatBetween 0.0 1.0)
            match kind with 
            | Nope -> return ms
            | Offset sigma ->
                let! ls    = Gen.arrayOfLength ms.Length (nextGaussian sigma)
                let! ans   = Gen.arrayOfLength ms.Length reasonableAngleRad
                let offs = 
                    Array.map2 (fun (a : float) l ->
                        V2d(cos a, sin a) * l
                    ) ans ls
                return             
                    Array.map3 (fun f (l,r) off -> 
                        if f <= 1.0 then
                            if f <= 1.0 * 0.5 then
                                (l+off,r)
                            else
                                (l,r+off)                        
                        else (l,r)                    
                    ) fs ms offs
            | Garbage chance -> 
                let! garbs = Gen.arrayOfLength ms.Length (arbV2d (Box2d(V2d.NN,V2d.II)))
                return
                    Array.map3 (fun garb (l,r) f -> 
                        if f <= chance then
                            if f <= chance * 0.5 then
                                (garb,r)
                            else
                                (l,garb)                        
                        else (l,r)
                    ) garbs ms fs
            | OffsetAndGarbage (sigma,chance) -> 
                let! ls  = Gen.arrayOfLength ms.Length (nextGaussian sigma)
                let! ans = Gen.arrayOfLength ms.Length reasonableAngleRad
                let offs = 
                    Array.map2 (fun (a : float) l ->
                        V2d(cos a, sin a) * l
                    ) ans ls
                let g =               
                    Array.map3 (fun f (l,r) off -> 
                        if f <= 1.0 then
                            if f <= 1.0 * 0.5 then
                                (l+off,r)
                            else
                                (l,r+off)                        
                        else (l,r)                    
                    ) fs ms offs
                let! fs2 = Gen.arrayOfLength ms.Length (floatBetween 0.0 1.0)
                let! garbs = Gen.arrayOfLength ms.Length (arbV2d (Box2d(V2d.NN,V2d.II)))
                return
                    Array.map3 (fun garb (l,r) f -> 
                        if f <= chance then
                            if f <= chance * 0.5 then
                                (garb,r)
                            else
                                (l,garb)                        
                        else (l,r)
                    ) garbs g fs2
        }        

    let arbLineWithin (box : Box3d) =
        gen {
            let! o = arbV3d (box.ShrunkBy(0.01))
            let! d = arbDir
            let step = 2.0 * (max (max box.Size.X box.Size.Y) box.Size.Z) * d
            let ray1 = Ray3d(o + step, -d)
            let ray0 = Ray3d(o - step,  d)
            let mutable t0 = 0.0
            box.Intersects(ray0,&t0) |> ignore
            let mutable t1 = 0.0
            box.Intersects(ray1,&t1) |> ignore
            return Line3d(ray0.GetPointOnRay t0, ray1.GetPointOnRay t1)
        }    

    let dataBounds (cam : Camera) (scale : float) = 
        let c = cam.Location + cam.Forward * 3.25 * scale
        let e = V3d.III*scale
        Box3d(c-e,c+e)

    let arbQuadFacing45 (cam : Camera) (scale : float) =
        gen {
            let bounds = dataBounds cam scale
            let ray = Ray3d(cam.Location,cam.Forward)
            let ct = ray.GetTOfProjectedPoint(bounds.Center)
            let h = bounds.Size.Length / 3.0

            let! cd = floatBetween (ct-h) (ct+h)
            let c = cam.Location + cam.Forward * cd

            let cone = Cone3d(cam.Location, cam.Forward, Constant.PiQuarter)
            let circ = cone.GetCircle(1.0)
            let! ra = reasonableAngleRad
            let! rr = floatBetween 0.0 circ.Radius
            let circ2world = circ.Plane.GetPlaneSpaceTransform()
            let p = V3d(rr*cos(ra),rr*sin(ra),0.0) |> circ2world.Forward.TransformPos
            let! cent = arbV3d (Box3d(cam.Location+V3d.NNN*0.1, cam.Location+V3d.III*0.1))
            let n = (cent - p).Normalized
            
            let u = Vec.cross n V3d.OOI |> Vec.normalize |> ((*)scale)
            let v = Vec.cross u n       |> Vec.normalize |> ((*)scale)    
            
            let p0 = c - u - v
            let p1 = c - u + v
            let p2 = c + u + v
            let p3 = c + u - v

            return Quad3d(p0,p1,p2,p3)
        }


    let genRotModifier =
        gen {
            let! i = Gen.choose(2,3)
            match i with
            | 1 -> return RotModifier.Not
            | 2 -> return Normal
            | 3 -> 
                let! a = reasonableAngleRad
                return NormalAndRoll a
            | _ -> 
                return failwith "no"
        }

    let genCameraTrans (scale : float) =
        gen {
            let! i = Gen.choose(3,3)
            match i with
            | 0 -> return ArbCameraTrans.No
            | 1 -> 
                let sh = scale/2.0
                let! d = floatBetween -sh sh
                return AlongFw d
            | 2 -> 
                let sh = scale        
                let! dx = floatBetween -sh sh 
                let! dy = floatBetween -sh sh 
                return InPlane (V2d(dx,dy))
            | 3 -> 
                let sh = scale
                let! dir = arbDir
                let! len = floatBetween -sh sh
                return ArbitraryDir (dir*len)
            | _ -> 
                return failwith "no"
        }


    let genArbPoints (cam : Camera) (scale : float) =
        gen {
            let bounds = dataBounds cam scale
            let! i = Gen.choose(0,4)
            match i with
            | 0 -> 
                let! q = arbQuadFacing45 cam scale 
                let! t = floatBetween 0.001 0.2
                return AlmostLinearQuad(q,t) 
            | 1 -> 
                let! q = arbQuadFacing45 cam scale             
                return InQuad q     
            | 2 -> 
                let! t = floatBetween 0.005 0.2
                return AlmostLinearVolume(bounds,t)
            | 3 -> 
                let! t = floatBetween 0.0001 0.2
                return AlmostFlatVolume(bounds,t)
            | 4 -> 
                return InVolume bounds    
            | _ -> 
                return failwith "no2"
        }

    let flattenQuad (q : Quad3d) (f : float) =
        
        let p = q.ComputeCentroid()
        let n = q.Normal             |> Vec.normalize
        let dy = Vec.cross n V3d.OOI |> Vec.normalize
        let dx = Vec.cross dy n      |> Vec.normalize
        let t = Euclidean3d(Rot3d.FromM33d(M33d.FromCols(dx,dy,n)), p)

        let qp0 = t.InvTransformPos(q.P0)
        let qp1 = t.InvTransformPos(q.P1)
        let qp2 = t.InvTransformPos(q.P2)
        let qp3 = t.InvTransformPos(q.P3)

        let minx = min (min (min qp0.X qp1.X) qp2.X) qp3.X
        let maxx = max (max (max qp0.X qp1.X) qp2.X) qp3.X
        let miny = min (min (min qp0.Y qp1.Y) qp2.Y) qp3.Y
        let maxy = max (max (max qp0.Y qp1.Y) qp2.Y) qp3.Y

        let rady = (maxy - miny)/2.0
        let cy = (miny + maxy)/2.0
        let newminy = cy - f*rady
        let newmaxy = cy + f*rady

        let np0 = V3d(minx,newminy,0.0) |> t.TransformPos
        let np1 = V3d(minx,newmaxy,0.0) |> t.TransformPos
        let np2 = V3d(maxx,newmaxy,0.0) |> t.TransformPos
        let np3 = V3d(maxx,newminy,0.0) |> t.TransformPos

        let nq = Quad3d(np0,np1,np2,np3)
        nq

    let flattenBox (b : Box3d) (f : float) =
        let s = V3d(f,1.0,1.0)
        b.ScaledFromCenterBy(s)

    let linearizeBox (b : Box3d) (f : float) =
        let s = V3d(f,2.0*f,1.0)
        b.ScaledFromCenterBy(s)

    let rec genPoints (a : ArbPoints) (ct : int) =
        gen {
            match a with
            | AlmostLinearQuad(q,lineness) ->   
                let nq = flattenQuad q lineness 
                return! genPoints (InQuad nq) ct

            | AlmostFlatVolume(b,flatness) -> 
                let nb = flattenBox b flatness
                return! genPoints (InVolume nb) ct
            | AlmostLinearVolume(b,lineness) -> 
                let nb = linearizeBox b lineness
                return! genPoints (InVolume nb) ct

            | InQuad q -> 
                let p = q.ComputeCentroid()
                let n = q.Normal             |> Vec.normalize
                let dy = Vec.cross n V3d.OOI |> Vec.normalize
                let dx = Vec.cross dy n      |> Vec.normalize
                let t = Euclidean3d(Rot3d.FromM33d(M33d.FromCols(dx,dy,n)), p)

                let qp0 = t.InvTransformPos(q.P0)
                let qp1 = t.InvTransformPos(q.P1)
                let qp2 = t.InvTransformPos(q.P2)
                let qp3 = t.InvTransformPos(q.P3)

                let minx = min (min (min qp0.X qp1.X) qp2.X) qp3.X
                let maxx = max (max (max qp0.X qp1.X) qp2.X) qp3.X
                let miny = min (min (min qp0.Y qp1.Y) qp2.Y) qp3.Y
                let maxy = max (max (max qp0.Y qp1.Y) qp2.Y) qp3.Y

                let qct = float ct/float 4 |> ceil |> int
                let! bla = Gen.arrayOfLength qct (arbV3d (Box3d(V3d(minx,miny,0.0), V3d(0.0,0.0,0.0))))
                let  bl  = bla |> Array.map t.TransformPos
                let! bra = Gen.arrayOfLength qct (arbV3d (Box3d(V3d(0.0,miny,0.0), V3d(maxx,0.0,0.0))))
                let  br  = bra |> Array.map t.TransformPos
                let! tla = Gen.arrayOfLength qct (arbV3d (Box3d(V3d(minx,0.0,0.0), V3d(0.0,maxy,0.0))))
                let  tl  = tla |> Array.map t.TransformPos
                let! tra = Gen.arrayOfLength qct (arbV3d (Box3d(V3d(0.0,0.0,0.0), V3d(maxx,maxy,0.0))))
                let  tr  = tra |> Array.map t.TransformPos
                
                return Array.concat [|bl;br;tr;tl|]
            | InVolume b -> 
                let bct = float ct/float 8 |> ceil |> int
                let Nx = b.Min.X
                let Ny = b.Min.Y
                let Nz = b.Min.Z
                let Ix = b.Max.X
                let Iy = b.Max.Y
                let Iz = b.Max.Z
                let Ox = (Ix + Nx) / 2.0
                let Oy = (Iy + Ny) / 2.0
                let Oz = (Iz + Nz) / 2.0

                let inline bb (x0 : float) y0 z0 (x1 : float) y1 z1 = Box3d(V3d(x0,y0,z0),V3d(x1,y1,z1))
                let b0 = bb Nx Oy Nz Ox Iy Oz
                let b1 = bb Nx Oy Oz Ox Iy Iz
                let b2 = bb Ox Oy Oz Ix Iy Iz
                let b3 = bb Ox Oy Nz Ix Iy Oz
                let b4 = bb Nx Ny Nz Ox Oy Oz
                let b5 = bb Nx Ny Oz Ox Oy Iz
                let b6 = bb Ox Ny Oz Ix Oy Iz
                let b7 = bb Ox Ny Nz Ix Oy Oz

                let! b0a = Gen.arrayOfLength bct (arbV3d b0)
                let! b1a = Gen.arrayOfLength bct (arbV3d b1)
                let! b2a = Gen.arrayOfLength bct (arbV3d b2)
                let! b3a = Gen.arrayOfLength bct (arbV3d b3)
                let! b4a = Gen.arrayOfLength bct (arbV3d b4)
                let! b5a = Gen.arrayOfLength bct (arbV3d b5)
                let! b6a = Gen.arrayOfLength bct (arbV3d b6)
                let! b7a = Gen.arrayOfLength bct (arbV3d b7)

                let ps = Array.concat [|b0a; b1a; b2a; b3a; b4a; b5a; b6a; b7a|]
                return ps
        }            

    let applyTrans (t : ArbCameraTrans) (c0 : Camera) =
        match t with
        | ArbCameraTrans.No -> c0
        | AlongFw len -> 
            let p1 = c0.Location + c0.Forward * len
            { c0 with view = CameraView.lookAt p1 (p1 + c0.Forward) c0.Up }
        | InPlane d -> 
            let p1 = c0.Location + c0.Right * d.X + c0.Up * d.Y
            { c0 with view = CameraView.lookAt p1 (p1 + c0.Forward) c0.Up }
        | ArbitraryDir v -> 
            let p1 = c0.Location + v
            { c0 with view = CameraView.lookAt p1 (p1 + c0.Forward) c0.Up }

    let applyRot (dataBb : Box3d) (r : RotModifier) (c0 : Camera) =
        match r with
        | Not -> c0
        | Normal -> 
            let fw1 = (dataBb.Center - c0.Location).Normalized
            { c0 with view = CameraView.lookAt c0.Location (c0.Location + fw1) c0.Up }
        | NormalAndRoll a -> 
            let fw1 = (dataBb.Center - c0.Location).Normalized
            let up1 = Rot3d(a,fw1).Transform c0.Up
            { c0 with view = CameraView.lookAt c0.Location (c0.Location + fw1) up1 }

    type ScenarioData =
        {   
            cam0 : Camera
            cam1 : Camera
            matches : (V2d*V2d)[]
            pts3d : V3d[]

            noise : NoiseKind
            points : ArbPoints
            camtrans : ArbCameraTrans
            camrot : RotModifier
        }

    type Scenario =
        | HomographyScenario of ScenarioData
        | FundamentalScenario of ScenarioData

    module Scenario =
        let getData = function HomographyScenario d -> d | FundamentalScenario d -> d

    let genScenarioData (pointcount : int) (noise : bool) (cam0 : Camera) (scale : float) (apts : ArbPoints) =
        gen {

            let! pts3d = genPoints apts pointcount
            let bb = dataBounds cam0 scale

            let! trans1 = genCameraTrans scale
            let! rot1 = genRotModifier 
            
            let cam1 = 
                cam0 |> applyTrans trans1
                     |> applyRot bb rot1
            let obs0 =
                pts3d |> Array.map (fun p -> Camera.projectUnsafe p cam0)
            let obs1 =
                pts3d |> Array.map (fun p -> Camera.projectUnsafe p cam1)

            let! noise = 
                if noise then genNoiseKind
                else Gen.constant Nope

            let nm = Array.zip obs0 obs1
            let! ms = applyNoise noise nm

            return {
                    cam0        = cam0
                    cam1        = cam1
                    matches     = ms
                    pts3d       = pts3d

                    points      = apts
                    camtrans    = trans1
                    camrot      = rot1
                    noise       = noise
                }
        }

    let genVolumeScenario =
        gen {
            let! scale = floatBetween 4.0 4.0   

            let p0 = V3d.OOO
            let t0 = V3d.IOO
            let fw0 = (t0 - p0).Normalized
            let nf = Trafo3d.FromNormalFrame(p0,fw0)
            let u0 = nf.Backward.C0.XYZ.Normalized
            let cv0 = CameraView.lookAt p0 t0 u0
            let! proj0 = arbProjection
            let cam0 = { view = cv0; proj = proj0 }
            let apts = 
                let bounds = dataBounds cam0 scale
                InVolume bounds

            let! ct = intBetween 128 256
            let! data = genScenarioData ct true cam0 scale apts
            let scenario = FundamentalScenario data
            return scenario
        }    

    let genPlaneScenario =
        gen {
            let! scale = floatBetween 4.0 4.0   

            let p0 = V3d.OOO
            let t0 = V3d.IOO
            let fw0 = (t0 - p0).Normalized
            let nf = Trafo3d.FromNormalFrame(p0,fw0)
            let u0 = nf.Backward.C0.XYZ.Normalized
            let cv0 = CameraView.lookAt p0 t0 u0
            let! proj0 = arbProjection
            let cam0 = { view = cv0; proj = proj0 }
            
            let! q = arbQuadFacing45 cam0 scale             
            let apts = InQuad q

            let! ct = intBetween 128 256
            let! data = genScenarioData ct true cam0 scale apts
            let scenario = HomographyScenario data
            return scenario
        }    
    let private generateScenario noise  =
        gen {
            let! scale = floatBetween 4.0 4.0   

            let p0 = V3d.OOO
            let t0 = V3d.IOO
            let fw0 = (t0 - p0).Normalized
            let nf = Trafo3d.FromNormalFrame(p0,fw0)
            let u0 = nf.Backward.C0.XYZ.Normalized
            let cv0 = CameraView.lookAt p0 t0 u0
            let! proj0 = arbProjection
            let cam0 = { view = cv0; proj = proj0 }
            let! apts = genArbPoints cam0 scale

            let! ct = intBetween 128 256
            let! data = genScenarioData ct noise cam0 scale apts
            let! scenario = 
                match apts with
                | InVolume _ -> FundamentalScenario data |> Gen.constant
                | InQuad _ | AlmostLinearQuad _ -> HomographyScenario data |> Gen.constant
                | AlmostFlatVolume (_,f) when f > 0.1 -> FundamentalScenario data |> Gen.constant
                | AlmostLinearVolume (_,f) when f > 0.05 -> FundamentalScenario data |> Gen.constant
                | _ -> 
                    gen {
                        let! p = floatBetween 0.0 1.0
                        if p <= 0.5 then return FundamentalScenario data else return HomographyScenario data                        
                    }
            return scenario
        }    

    let genScenario = generateScenario true
    let genNoiselessScenario = generateScenario false