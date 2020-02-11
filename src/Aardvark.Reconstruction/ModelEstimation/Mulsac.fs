namespace Aardvark.Reconstruction


// Naive multi-objective Ransac implementation using "find and remove"-style solution space traversal strategy.


open System
open Aardvark.Base
open System.Threading.Tasks
open System.Threading
open System.Linq
open RansacHelpers

[<AutoOpen>]
module Mulsac =

    let inline takeRandomIndices (rand : System.Random) (ws : float[]) (k : int) (d : int[])=

        let ws = ws |> Array.copy
        let n = ws.Length
        if k > n || k < 0 then 
            failwithf "cannot take %d elements out of %d" k n

        elif k = n then
            for i in 0 .. k - 1 do d.[i] <- i

        else
            let mutable rand = rand
            let mutable bad = true
            while bad do
                let mutable iter = 0
                let mutable cnt = 0
                while cnt < k && iter < d.Length * 4 do
                    let r = rand.NextDouble() * (ws |> Array.sum)
                    let mutable found = false
                    let mutable cdf = 0.0
                    let mutable i = 0
                    while not found do
                        cdf <- cdf + ws.[i]
                        if r <= cdf then
                            d.[cnt] <- i
                            ws.[i] <- 0.0
                            cnt <- cnt + 1
                            iter <- iter + 1
                            found <- true
                        else                        
                            i <- i+1

                if cnt = k then
                    bad <- false
                else
                    Log.warn "random is bad"
                    rand <- Random()

type Thing<'a,'b> =
    struct
        val inlierCount : int
        val index : int[]
        val ils : 'a[]
        val b : 'b

        new(i,id,il,b) = { inlierCount = i; index = id; ils = il; b = b }
    end
   
type Mulsac =
    static member solve<'a, 'b, 't>(p : float, w : float, needed : int, construct :  'a[] -> list<'b>, 
                                    countInliers : 'b -> 't[] -> int,  getInliers : 'b -> 't[] -> int[], 
                                    train : 'a[], test : 't[], weights : float[], same : 'b -> 'b -> 'a[] -> 'a[] -> bool ) : seq<_> =
        let inlier = OptimizedClosures.FSharpFunc<'b, 't[], int>.Adapt countInliers
        
        let solct = 16
        
        let cmp = {
            new System.Collections.Generic.IComparer<Thing<'a,'b>> with
                member x.Compare(l : Thing<'a,'b>, r : Thing<'a,'b>) =
                    -(compare l.inlierCount r.inlierCount)
        }

        let sortAndPrune (bS : System.Collections.Generic.List<Thing<'a,'b>>)  = 
            if bS.Count > 0 then

                bS.Sort(cmp)

                let largest = bS.[0]
                let target = float largest.inlierCount * 0.75
                let s = 
                    let mutable found = false
                    let mutable i = 0
                    while not found && i < bS.Count do
                        let b = bS.[i]
                        if float b.inlierCount < target then
                            found <- true
                        else
                            i <- i + 1
                    i
                let n = bS.Count - s
                if n > 0 && s >= 0 then
                    bS.RemoveRange(s,n)

                if bS.Count > solct then
                    let n = bS.Count - solct
                    bS.RemoveRange(solct,n)

                bS.[bS.Count-1].inlierCount
            else
                failwith "can not happen"

        let enqueue (bS : System.Collections.Generic.List<Thing<'a,'b>>) (thing : Thing<'a,'b>) =
            let mutable found = false
            for i in 0..bS.Count-1 do
                let s = bS.[i]
                if same thing.b s.b thing.ils s.ils then
                    found <- true
                    if thing.inlierCount > s.inlierCount then
                        bS.[i] <- thing

            if not found then
                bS.Add(thing)

            sortAndPrune bS

        timed (fun () ->

            if train.Length < needed then
                [] :> seq<_>

            elif train.Length = needed then
                let mutable bestSol = Unchecked.defaultof<_>
                let mutable bestCnt = 0
                let sol = construct train
                match sol with
                    | [] -> 
                        ()
                    | sols ->
                        for s in sols do
                            let cnt = inlier.Invoke(s, test) 
                            if cnt >= needed && cnt > bestCnt then
                                bestSol <- s
                                bestCnt <- cnt
                        

                if bestCnt = 0 then
                    [] :> seq<_>
                else
                    [ {
                        data = train
                        test = test
                        value = bestSol
                        modelIndices = Array.init train.Length id
                        inliers = getInliers bestSol test
                        iterations = 1
                        time = MicroTime.Zero
                    }] :> seq<_>

            else
                let p = clamp 0.1 0.999999 p
                let w = clamp 0.01 0.99 w
            
                // https://en.wikipedia.org/wiki/Random_sample_consensus#Parameters
                // the number of iterations must be nonzero
                let iter = log (1.0 - p) / log (1.0 - pown w needed) |> ceil |> int |> max 1

                let l = obj()
                let mutable bestCount = -1
                let mutable bestSol   = []
                    
                let mutable total = iter
                let mutable finished = 0

                let rand = System.Random()
                    
                Parallel.For(0, Environment.ProcessorCount, fun pi ->
                    let rand = lock rand (fun () -> System.Random(rand.Next() + Thread.CurrentThread.ManagedThreadId))
                    let input = Array.zeroCreate needed
                    let mutable bC = -1
                    let bS = System.Collections.Generic.List<Thing<_,_>>(solct+1)
                    let index = Array.zeroCreate needed

                    let inline enqueue b ilct ils (f : int -> unit) =
                        let lowest = enqueue bS (Thing<_,_>(ilct,index |> Array.copy,ils,b))
                        f lowest
                        
                    let mutable good = 0
                    let mutable bad = 0
                    let mutable failed = false
                    let inline reasonable() =
                        if bad > 1000 then
                            failed <- true
                            //Log.warn "too much badness"
                            false
                        else
                            true

                    while (finished < total && reasonable()) do
                        Mulsac.takeRandomIndices rand weights needed index

                        // construct a new solution
                        for i in 0 .. needed-1 do 
                            input.[i] <- train.[index.[i]]
                        let sol = input |> construct
                        match sol with
                        | [] ->
                            bad <- bad + 1

                        | sols -> 
                            Interlocked.Increment(&finished) |> ignore

                            for sol in sols do
                                // count inliers
                                let inlierCount = inlier.Invoke(sol, test)

                                // store the best solution
                                if inlierCount > bC then
                                    enqueue sol inlierCount (input |> Array.copy) (fun lowest -> bC <- lowest )

                            good <- good + 1
                            bad <- 0
                            
                    if not failed then
                        lock l (fun () ->
                            // store the best solution
                            if bC > bestCount then
                                bestCount <- bC
                                bestSol <- (bS |> Seq.toList)::bestSol
                        )
                ) |> ignore

                let bestSols =
                    let bS = System.Collections.Generic.List<Thing<_,_>>(solct+1)
                    bestSol |> List.concat |> List.iter (fun t -> enqueue bS t |> ignore)
                    bS :> seq<_>


                if bestCount < 0 then
                    [] :> seq<_>
                else
                    bestSols |> Seq.map( fun bestSol -> 
                        let indices = getInliers bestSol.b test
                        {
                            data = train
                            test = test
                            value = bestSol.b
                            modelIndices = bestSol.index
                            inliers = indices
                            iterations = iter
                            time = MicroTime.Zero
                        }
                    )
        ) 
        
type MulsacProblem<'d, 't, 'i, 's> =
    {
        neededSamples   : int
        solve           : 'd[] -> list<'i>
        countInliers    : 'i -> 't[] -> int
        getInliers      : 'i -> 't[] -> int[]
        same            : 'i -> 'i -> 'd[] -> 'd[] -> bool
        getSolution     : RansacResult<'d, 't, 'i> -> 's
    }
    member x.SolveMap(  cfg : RansacConfig, 
                        project : seq<RansacResult<'d, 't, 's>> -> seq<RansacResult<'d, 't, 's>>, 
                        train : 'd[], 
                        weights : float[], 
                        test : 't[]) =

        let result = 
            Mulsac.solve(
                cfg.probability, cfg.expectedRelativeCount,
                x.neededSamples,
                x.solve,
                x.countInliers,
                x.getInliers,
                train,
                test,
                weights,
                x.same
            )
        result |> Seq.map ( fun res ->
            {
                data            = res.data
                test            = res.test
                value           = x.getSolution res
                modelIndices    = res.modelIndices
                inliers         = res.inliers
                iterations      = res.iterations
                time            = res.time
            }
        )   |> project

    member x.Solve( cfg : RansacConfig, 
                    train : 'd[], 
                    weights : float[], 
                    test : 't[]) =
        x.SolveMap(cfg,id,train,weights,test)

    interface IRansacProblem<'d, 't, 's> with
        member x.Solve(cfg, train, test) = x.Solve(cfg,train,Array.replicate train.Length 1.0,test)
        