namespace Aardvark.Reconstruction

open Aardvark.Base

[<Struct; CustomEquality; NoComparison>]
type Distortion2d(k1 : float, k2 : float, k3 : float, p1 : float, p2 : float) =

    member x.K1 = k1
    member x.K2 = k2
    member x.K3 = k3
    member x.P1 = p1
    member x.P2 = p2

    static member Identity = Distortion2d(0.0, 0.0, 0.0, 0.0, 0.0)

    override x.GetHashCode() =
        HashCode.Combine(hash k1, hash k2, hash k3, hash p1, hash p2)
        
    override x.Equals(o : obj) =    
        match o with
        | :? Distortion2d as o ->
            k1 = o.K1 && k2 = o.K2 && k3 = o.K3 && p1 = o.P1 && p2 = o.P2
        | _ ->
            false
            
    override x.ToString() =
        sprintf "Distortion2d { k1 = %g, k2 = %g, k3 = %g, p1 = %g, p2 = %g }" k1 k2 k3 p1 p2

    member x.TransformPos(pos : V2d) =
        let r2 = pos.LengthSquared
        let r4 = sqr r2
        let r6 = r4 * r2
        let factor = 1.0 + k1*r2 + k2*r4 + k3*r6
        
        let tangential =
            V2d(
                2.0*p1*pos.X*pos.Y + p2*(r2 + 2.0*sqr pos.X), 
                p1*(r2 + 2.0*sqr pos.Y) + 2.0*p2*pos.X*pos.Y
            )

        pos * factor + tangential
        
    member x.InvTransformPos(pos : V2d, maxIter : int) =
        let mutable res = pos
        let mutable iter = 0
        while iter < maxIter && (x.TransformPos res - pos).Norm1 > 1E-8 do
            let distPart =
                let r2 = res.LengthSquared
                let r4 = sqr r2
                let r6 = r4 * r2
                let factor = k1*r2 + k2*r4 + k3*r6
        
                let tangential =
                    V2d(
                        2.0*p1*res.X*res.Y + p2*(r2 + 2.0*sqr res.X), 
                        p1*(r2 + 2.0*sqr res.Y) + 2.0*p2*res.X*res.Y
                    )

                res * factor + tangential
                
            res <- pos - distPart
            iter <- iter + 1

        res
        
    member x.InvTransformPos(pos : V2d) =
        x.InvTransformPos(pos, 15)
        
