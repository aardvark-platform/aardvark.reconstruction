namespace Aardvark.Reconstruction

open Aardvark.Base
open FSharp.Data
open System.IO

type private SensorCsv = CsvProvider<".\\database.csv", Separators = ",", ResolutionFolder = __SOURCE_DIRECTORY__>
type private AliceVisionCsv = CsvProvider<"Maker;Model;SensorWidth\r\n10.or;10.or D2;4.73", Separators = ";", ResolutionFolder = __SOURCE_DIRECTORY__, AssumeMissingValues = true, HasHeaders = true, IgnoreErrors = true>
type private Marker = Marker

module Sensors =
    open MetadataExtractor
    open MetadataExtractor.Formats.Exif

    let private parseCsv (file : Stream) =
        let r = new StreamReader(file)
        let res = System.Collections.Generic.List()
        while not r.EndOfStream do
            let toks = r.ReadLine().Split([|';'|])
            if toks.Length >= 3 then    
                let maker = toks.[0].Trim().ToLower()
                let model = toks.[1].Trim().ToLower()
                match System.Double.TryParse(toks.[2].Trim().ToLower(), System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture) with
                | (true,v) -> 
                    res.Add(maker,model,v)
                | _ -> ()
        res

    let private data = 
        let data = SensorCsv.Load(typeof<Marker>.Assembly.GetManifestResourceStream "Aardvark.Reconstruction.SensorDb.database.csv")
        let res = System.Collections.Generic.Dictionary<string, Map<string, V2d * V2i>>()

        // for r in data.Rows do
        //     if r.``SensorWidth(pixels)``.HasValue && r.``SensorHeight(pixels)``.HasValue then
        //         let s = V2d(float r.``SensorWidth(mm)``, r.``SensorHeight(mm)``)
        //         let ps = V2i(r.``SensorWidth(pixels)``.Value, r.``SensorHeight(pixels)``.Value)

        //         let key = r.CameraMaker.Trim().ToLower()
        //         let model = r.CameraModel.Trim().ToLower()

        //         let map =
        //             match res.TryGetValue key with
        //             | (true, m) -> m
        //             | _ -> Map.empty

        //         res.[key] <- Map.add model (s, ps) map

        res
        
    let private avData =
        let res1 = System.Collections.Generic.Dictionary<string, Map<string, float>>()
        let data1 = parseCsv(typeof<Marker>.Assembly.GetManifestResourceStream "Aardvark.Reconstruction.SensorDb.aliceVision.csv")
        
        for (key,model,sw) in data1 do
            let map =
                match res1.TryGetValue key with
                | (true, m) -> m
                | _ -> Map.empty

            res1.[key] <- Map.add model sw map
        res1

    let private regex = System.Text.RegularExpressions.Regex @"[- \t]+"
    let private matching (a : string) (b : string) =
        let ac = regex.Split(a.Trim().ToLower())
        let bc = regex.Split(b.Trim().ToLower())
        let sa = Set.ofArray ac
        let sb = Set.ofArray bc
        Set.intersect sa sb

    let private widthKeys =
        [
            ExifIfd0Directory.TagImageWidth
            ExifImageDirectory.TagExifImageWidth
            ExifSubIfdDirectory.TagRelatedImageWidth
        ]
        
    let private heightKeys =
        [
            ExifIfd0Directory.TagImageHeight
            ExifImageDirectory.TagExifImageHeight
            ExifSubIfdDirectory.TagRelatedImageHeight
        ]
        
    let private lookupData (maker : string) (model : string) (focal : float) (w : Option<int>) (h : Option<int>) =
        let maker = maker.Trim().ToLower().Replace("corporation", "").Trim()
        match data.TryGetValue(maker.Trim().ToLower()) with
        | (true, map) ->    
            let model = model.Trim().ToLower().Replace(maker.Trim().ToLower(), "").Trim()
            let matchingModels =
                map |> Map.toList |> List.choose (fun (mi, (s, ps)) ->
                    let o = matching model mi
                    if not (Set.isEmpty o) then
                        let imageSize = 
                            match w, h with
                            | Some w, Some h -> V2i(w,h)
                            | _ -> ps
                        let fp = (float (max ps.X ps.Y) * float focal) / s.X
                        Some (o, imageSize, 2.0 * fp / float imageSize.Y)
                    else
                        None

                )

            matchingModels |> List.sortByDescending (fun (s,a,b) -> Set.count s) |> List.tryHead |> Option.map (fun (a,b,c) -> b,c)
        | _ -> 
            None

    let private lookupAlice (maker : string) (model : string) (focal : float)  (w : int) (h : int) =
        let maker = maker.Trim().ToLower().Replace("corporation", "").Trim()
        match avData.TryGetValue(maker.Trim().ToLower()) with
        | (true, map) ->    
            let model = model.Trim().ToLower().Replace(maker.Trim().ToLower(), "").Trim()
            let matchingModels =
                map |> Map.toList |> List.choose (fun (mi, sw) ->
                    let o = matching model mi
                    if not (Set.isEmpty o) then
                        let imageSize = V2i(w,h)
                        let fp = (float (max w h) * float focal) / sw
                        Some (o, imageSize, 2.0 * fp / float imageSize.Y)
                    else
                        None

                )

            matchingModels |> List.sortByDescending (fun (s,a,b) -> Set.count s) |> List.tryHead |> Option.map (fun (a,b,c) -> b,c)
        | _ -> 
            None

    let private tryLookupExif (file : string) (w : Option<int>) (h : Option<int>) =
        let exif = MetadataExtractor.ImageMetadataReader.ReadMetadata(file)
        let exif = exif |> Seq.filter (fun e -> e.Name <> "PrintIM") |> Seq.toArray
        let w = 
            exif |> Seq.tryPick ( fun d -> 
                widthKeys |> List.tryPick (fun s ->  if d.ContainsTag s then Some (d.GetString s |> int) else None )
            )
            
        let h = 
            exif |> Seq.tryPick ( fun d -> 
                heightKeys |> List.tryPick (fun s ->  if d.ContainsTag s then Some (d.GetString s |> int) else None )
            )

        let tryGetInt (tag : int) =
            exif |> Seq.tryPick (fun d ->
                if d.ContainsTag tag then Some (d.GetInt32 tag)
                else None
            )
        let tryGetFloat (tag : int) =
            exif |> Seq.tryPick (fun d ->
                if d.ContainsTag tag then Some (d.GetSingle tag)
                else None
            )
        let tryGetString (tag : int) =
            exif |> Seq.tryPick (fun d ->
                if d.ContainsTag tag then Some (d.GetString tag)
                else None
            )

        match tryGetString ExifIfd0Directory.TagMake, tryGetString ExifIfd0Directory.TagModel, tryGetFloat ExifDirectoryBase.TagFocalLength with
        | Some maker, Some model, Some focal -> 
            printfn "%s %s %f" maker model focal

            match lookupData maker model (float focal) w h with
            | Some r -> Some r 
            | None -> 
                match w,h with 
                | Some w, Some h -> 
                    match lookupAlice maker model (float focal) w h with 
                    | Some r -> Some r
                    | None -> 
                        Log.warn "[SensorDb] No entry in databases found for %s %s %A" maker model focal
                        None
                | _ -> 
                    Log.warn "[SensorDb] No width/height in EXIF %s" file
                    None
        | _ ->
            Log.warn "[SensorDb] no EXIF in %s" file
            None

    /// Look up the focal length (physical focal length divided by physical width of sensor, in NDC) given make and model names, and the physical focal length in millimetres of an image, as well as its width and height.
    let tryGetFocalLength (maker : string) (model : string) (focalInMm : float) (w : int) (h : int) =
        match lookupData maker model (float focalInMm) (Some w) (Some h) with
            | Some r -> Some (snd r) 
            | None -> 
                match lookupAlice maker model (float focalInMm) w h with 
                | Some r -> Some (snd r)
                | None -> 
                    Log.warn "[SensorDb] No entry in databases found for %s %s %A (%dx%d)" maker model focalInMm w h
                    None

    /// Look up the image size and focal length from Exif.
    let tryGetFocalLengthFromExif (file : string) =
        tryLookupExif file None None
        
    /// Look up the focal length from Exif normalized to a specific image size.
    let tryGetFocalLengthFromExifCustomSize (file : string) (w : int) (h : int) =
        tryLookupExif file (Some w) (Some h)
