namespace Aardvark.Reconstruction

open Aardvark.Base


[<AutoOpen>]
module Utilities =
    /// returns the sign of the given value where `sign1(0) = 1`
    let inline sign1 v =
        if v >= LanguagePrimitives.GenericZero then LanguagePrimitives.GenericOne
        else -LanguagePrimitives.GenericOne

    type V3d with
        member a.OuterProduct(b : V3d) =
            M33d(
                a.X * b.X, a.X * b.Y, a.X * b.Z,
                a.Y * b.X, a.Y * b.Y, a.Y * b.Z,
                a.Z * b.X, a.Z * b.Y, a.Z * b.Z
            )
 
    type M33d with
        static member Diagonal(a : float, b : float, c : float) =
            M33d(
                a, 0.0, 0.0, 
                0.0, b, 0.0, 
                0.0, 0.0, c
            )

        member m.ToOrthoNormal() =
            let x = Vec.normalize m.C0
            let y = Vec.normalize m.C1
            let z = Vec.normalize m.C2

            let y1 = y - Vec.dot z y * z |> Vec.normalize
            let x1 = x - Vec.dot z x * z - Vec.dot y1 x * y1 |> Vec.normalize

            M33d.FromCols(x1,y1,z)

module Vec = 
    let outerProduct (a : V3d) (b : V3d) =
        a.OuterProduct b

module List =
    let rec atMost (n : int) (l : list<'a>) =
        if n <= 0 then []
        else 
            match l with
                | [] -> []
                | h :: t -> h :: atMost (n - 1) t
    
    module Parallel =
        open System.Linq

        let map (mapping : 'a -> 'b) (l : list<'a>) =
            l.AsParallel().AsOrdered().Select(mapping) |> Seq.toList
            
        let choose (mapping : 'a -> Option<'b>) (l : list<'a>) =
            l.AsParallel().AsOrdered().Select(mapping) |> Seq.choose id |> Seq.toList
            
        let collect (mapping : 'a -> list<'b>) (l : list<'a>) =
            l.AsParallel().AsOrdered().Select(mapping) |> List.concat


module PixImage = 
    open System

    type Lerper<'a> private() =
        static let lerp : float -> 'a -> 'a -> 'a =
            if typeof<'a> = typeof<uint8> then unbox <| fun (t : float) (a : uint8) (b : uint8) -> Fun.Lerp(t, a, b)
            elif typeof<'a> = typeof<uint16> then unbox <| fun (t : float) (a : uint16) (b : uint16) -> Fun.Lerp(t, a, b)
            elif typeof<'a> = typeof<uint32> then unbox <| fun (t : float) (a : uint32) (b : uint32) -> Fun.Lerp(t, a, b)
            else fun (t : float) a b -> if t < 0.5 then a else b

        static member Lerp = lerp

    [<AbstractClass>]
    type PixImageVisitor<'r>() =
        static let table =
            LookupTable.lookupTable [
                typeof<int8>, (fun (self : PixImageVisitor<'r>, img : PixImage) -> self.Visit<int8>(unbox img, 127y))
                typeof<uint8>, (fun (self : PixImageVisitor<'r>, img : PixImage) -> self.Visit<uint8>(unbox img, 255uy))
                typeof<int16>, (fun (self : PixImageVisitor<'r>, img : PixImage) -> self.Visit<int16>(unbox img, Int16.MaxValue))
                typeof<uint16>, (fun (self : PixImageVisitor<'r>, img : PixImage) -> self.Visit<uint16>(unbox img, UInt16.MaxValue))
                typeof<int32>, (fun (self : PixImageVisitor<'r>, img : PixImage) -> self.Visit<int32>(unbox img, Int32.MaxValue))
                typeof<uint32>, (fun (self : PixImageVisitor<'r>, img : PixImage) -> self.Visit<uint32>(unbox img, UInt32.MaxValue))
                typeof<int64>, (fun (self : PixImageVisitor<'r>, img : PixImage) -> self.Visit<int64>(unbox img, Int64.MaxValue))
                typeof<uint64>, (fun (self : PixImageVisitor<'r>, img : PixImage) -> self.Visit<uint64>(unbox img, UInt64.MaxValue))
                typeof<float16>, (fun (self : PixImageVisitor<'r>, img : PixImage) -> self.Visit<float16>(unbox img, float16(Float32 = 1.0f)))
                typeof<float32>, (fun (self : PixImageVisitor<'r>, img : PixImage) -> self.Visit<float32>(unbox img, 1.0f))
                typeof<float>, (fun (self : PixImageVisitor<'r>, img : PixImage) -> self.Visit<float>(unbox img, 1.0))
            ]
        abstract member Visit<'a when 'a : unmanaged> : PixImage<'a> * 'a -> 'r

        interface IPixImageVisitor<'r> with
            member x.Visit<'a>(img : PixImage<'a>) =
                table (typeof<'a>) (x, img)    

    let downsample (factor : float) (img : PixImage) =
        if factor >= 1.0 then 
            img
        else
            let size = V2i(max 1 (int (factor * float img.Size.X)), max 1 (int (factor * float img.Size.Y)))
            img.Visit {
                new PixImageVisitor<PixImage>() with
                    member x.Visit<'a when 'a : unmanaged>(img : PixImage<'a>, def : 'a) : PixImage =
                        let dst = PixImage<'a>(img.Format, size)
                        NativeVolume.using img.Volume (fun pSrc ->
                            NativeVolume.using dst.Volume (fun pDst ->
                                pSrc.BlitTo(pDst, Lerper<'a>.Lerp)
                                dst :> PixImage
                            )
                        )
            }


type ImagePoint =
    struct
        val mutable ndc : V2d
        val mutable idx : int
    end

type CameraMatch =
    {
        tag : string
        motion  : CameraMotion
        matches : array<ImagePoint * ImagePoint>
    }

    static member Combine(l : CameraMatch, r : CameraMatch) =
            
        let matches = 
            l.matches |> Array.choose (fun (lp,mp) ->
                let rp = r.matches |> Array.tryPick (fun (mpi,rp) -> if mp.idx = mpi.idx then Some rp else None)
                match rp with
                    | Some rp -> Some (lp,rp)
                    | None -> None
            )

        if matches.Length > 0 then
            Some {
                tag = l.tag + " >> " + r.tag
                motion = l.motion + r.motion
                matches = matches
            }
        else
            None

    member x.Inverse =
        let tag = 
            if x.tag.Contains ">" then "(" + x.tag + ")^-1" 
            else x.tag + "^-1" 
        {
            tag = tag
            motion = x.motion.Inverse
            matches = x.matches |> Array.map (fun (l,r) -> r,l)
        }

module M33d =
    let inline crossProductMatrix (v : V3d) =
        M33d(
            0.0, -v.Z,  v.Y,
            v.Z,  0.0, -v.X,
            -v.Y, v.X,  0.0
        )