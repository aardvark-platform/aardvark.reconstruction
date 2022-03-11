namespace LensFunSharp

open System
open Silk.NET.Core
open Silk.NET.Core.Loader
open Silk.NET.Core.Native
open System.Runtime.InteropServices
open System.Runtime.CompilerServices
open System.Security
open Microsoft.FSharp.NativeInterop
open Aardvark.Base
open System.IO
open System.IO.Compression
open System.Net

#nowarn "9"

[<Flags>]
type SearchFlags =
    | None = 0
    | Loose = 1
    | SortAndUniquify = 2

type LensType =
    | Unknown = 0
    | Rectilinear = 1
    | Fisheye = 2
    | Panoramic = 3 
    | Equirectangular = 4
    | FisheyeOrthographic = 5
    | FisheyeStereographic = 6
    | FisheyeEquisolid = 7
    | FisheyeThoby = 8

type DistortionModel =
    /// Distortion parameters are unknown.
    | None = 0
    /// 3rd order polynomial model, which is a subset of the PTLens model.
    | Poly3 = 1
    /// 5th order polynomial model.
    | Poly5 = 2
    /// PTLens model, which is also used by Hugin.
    | PtLens = 3
    /// Adobe Camera Model. The coordinate system is different here. Everything is measured in units of the focal length of the lens.
    | Acm = 4


module Implementation = 

    type LensFunPixelFormat =
        | U8 = 0
        | U16 = 1
        | U32 = 2
        | F32 = 3
        | F64 = 4

    [<StructLayout(LayoutKind.Sequential)>]
    type LensFunModifierHandle =
        struct
            val mutable public Handle : nativeint
        end

    [<StructLayout(LayoutKind.Sequential)>]
    type LensCalibAttributes =
        struct
            val mutable public CropFactor : float32
            val mutable public Aspect : float32
            val mutable public CenterX : float32
            val mutable public CenterY : float32
        end

    [<StructLayout(LayoutKind.Sequential)>]
    type LensCalibDistortionHandle =
        struct
            val mutable public Model : DistortionModel
            val mutable public Focal : float32
            val mutable public RealFocal : float32
            val mutable public RealFocalMeasured : int
            val mutable public P0 : float32
            val mutable public P1 : float32
            val mutable public P2 : float32
            val mutable public P3 : float32
            val mutable public P4 : float32
            val mutable public Attributes : LensCalibAttributes
        end

    [<StructLayout(LayoutKind.Sequential)>]
    type LensFunDatabaseHandle =
        struct
            val mutable public Handle : nativeint

            member x.IsValid = x.Handle <> 0n
            member x.IsNull = x.Handle = 0n
            static member Null = LensFunDatabaseHandle 0n
            new(h) = { Handle = h }
        end
    
    [<StructLayout(LayoutKind.Sequential)>]
    type LensFunCameraHandle =
        struct
            val mutable public Maker        : nativeptr<byte>
            val mutable public Model        : nativeptr<byte>
            val mutable public Variant      : nativeptr<byte>
            val mutable public Mount        : nativeptr<byte>
            val mutable public CropFactor   : float32
            val mutable public Score        : int
        end
        
    [<StructLayout(LayoutKind.Sequential)>]
    type LensFunMountHandle =
        struct
            val mutable public Name     : nativeptr<byte>
            val mutable public Compat   : nativeptr<nativeptr<byte>>
        end
        
    [<StructLayout(LayoutKind.Sequential)>]
    type LensFunLensHandle =
        struct
            val mutable public Maker        : nativeptr<byte>
            val mutable public Model        : nativeptr<byte>
            val mutable public MinFocal     : float32
            val mutable public MaxFocal     : float32
            val mutable public MinAperture  : float32
            val mutable public MaxAperture  : float32
            val mutable public Mounts       : nativeptr<nativeptr<byte>>
            val mutable public Type         : LensType
            val mutable public CenterX      : float32
            val mutable public CenterY      : float32
            val mutable public CropFactor   : float32
            val mutable public AspectRatio  : float32
            val mutable public CalibDistortion  : nativeint
            val mutable public CalibTCA         : nativeint
            val mutable public CalibVignetting  : nativeint
            val mutable public CalibCrop        : nativeint
            val mutable public CalibFov         : nativeint
            val mutable public Score            : int
        end


    type LensFunError =
        | NoError = 0
        | WrongFormat = 1


    module LensFunRaw =
        [<Literal>]
        let lib = "lensfun"

        [<DllImport(lib, EntryPoint = "lf_db_new"); SuppressUnmanagedCodeSecurity>]
        extern LensFunDatabaseHandle newDatabase()

        [<DllImport(lib, EntryPoint = "lf_db_load"); SuppressUnmanagedCodeSecurity>]
        extern void loadDatabase(LensFunDatabaseHandle db)

        [<DllImport(lib, EntryPoint = "lf_db_destroy"); SuppressUnmanagedCodeSecurity>]
        extern void destroyDatabase(LensFunDatabaseHandle db)

        [<DllImport(lib, EntryPoint = "lf_db_load_file", CharSet = CharSet.Ansi); SuppressUnmanagedCodeSecurity>]
        extern LensFunError loadDatabaseFile(LensFunDatabaseHandle db, string path)

        [<DllImport(lib, EntryPoint = "lf_db_load_data", CharSet = CharSet.Ansi); SuppressUnmanagedCodeSecurity>]
        extern LensFunError loadData(LensFunDatabaseHandle db, [<MarshalAs(UnmanagedType.LPStr)>] string errorContext, [<MarshalAs(UnmanagedType.LPStr)>] string data, nativeint dataSize) 

        [<DllImport(lib, EntryPoint = "lf_db_get_cameras"); SuppressUnmanagedCodeSecurity>]
        extern LensFunCameraHandle* * getCameras(LensFunDatabaseHandle db)
        
        [<DllImport(lib, EntryPoint = "lf_db_get_mounts"); SuppressUnmanagedCodeSecurity>]
        extern LensFunMountHandle* * getMounts(LensFunDatabaseHandle db)
        
        [<DllImport(lib, EntryPoint = "lf_db_get_lenses"); SuppressUnmanagedCodeSecurity>]
        extern LensFunLensHandle* * getLenses(LensFunDatabaseHandle db)
        
        [<DllImport(lib, EntryPoint = "lf_db_find_cameras", CharSet = CharSet.Ansi); SuppressUnmanagedCodeSecurity>]
        extern LensFunCameraHandle* * findCameras(LensFunDatabaseHandle db, [<MarshalAs(UnmanagedType.LPStr)>] string maker, [<MarshalAs(UnmanagedType.LPStr)>] string model)

        [<DllImport(lib, EntryPoint = "lf_db_find_cameras_ext", CharSet = CharSet.Ansi); SuppressUnmanagedCodeSecurity>]
        extern LensFunCameraHandle* * findCamerasExt(LensFunDatabaseHandle db, [<MarshalAs(UnmanagedType.LPStr)>] string maker, [<MarshalAs(UnmanagedType.LPStr)>] string model, SearchFlags flags)

        [<DllImport(lib, EntryPoint = "lf_db_find_lenses_hd", CharSet = CharSet.Ansi); SuppressUnmanagedCodeSecurity>]
        extern LensFunLensHandle* * findLenses(LensFunDatabaseHandle db, LensFunCameraHandle* camera, [<MarshalAs(UnmanagedType.LPStr)>] string maker, [<MarshalAs(UnmanagedType.LPStr)>] string lensModel, SearchFlags flags)

        [<DllImport(lib, EntryPoint = "lf_db_find_mount", CharSet = CharSet.Ansi); SuppressUnmanagedCodeSecurity>]
        extern LensFunMountHandle* findMount(LensFunDatabaseHandle db, [<MarshalAs(UnmanagedType.LPStr)>] string name)

        [<DllImport(lib, EntryPoint = "lf_lens_get_mount_names", CharSet = CharSet.Ansi); SuppressUnmanagedCodeSecurity>]
        extern byte* * getMountNames(LensFunDatabaseHandle db, LensFunLensHandle* lens)

        [<DllImport(lib, EntryPoint = "lf_lens_interpolate_distortion", CharSet = CharSet.Ansi); SuppressUnmanagedCodeSecurity>]
        extern int interpolateDistortion(LensFunLensHandle* lens, float32 crop, float32 focal, LensCalibDistortionHandle& distortion)


        [<DllImport(lib, EntryPoint = "lf_modifier_create", CharSet = CharSet.Ansi); SuppressUnmanagedCodeSecurity>]
        extern LensFunModifierHandle createModifier(LensFunLensHandle* lens, float32 focal, float32 imgCrop, int width, int height, LensFunPixelFormat format, bool reverse)
        
        [<DllImport(lib, EntryPoint = "lf_modifier_destroy", CharSet = CharSet.Ansi); SuppressUnmanagedCodeSecurity>]
        extern void destroyModifier(LensFunModifierHandle handle)
        
        [<DllImport(lib, EntryPoint = "lf_modifier_enable_scaling", CharSet = CharSet.Ansi); SuppressUnmanagedCodeSecurity>]
        extern int enableScaling(LensFunModifierHandle handle, float32 scale)
        
        [<DllImport(lib, EntryPoint = "lf_modifier_enable_distortion_correction", CharSet = CharSet.Ansi); SuppressUnmanagedCodeSecurity>]
        extern int enableDistortionCorrection(LensFunModifierHandle handle)
        
        [<DllImport(lib, EntryPoint = "lf_modifier_enable_tca_correction", CharSet = CharSet.Ansi); SuppressUnmanagedCodeSecurity>]
        extern int enableTcaCorrection(LensFunModifierHandle handle)
        
        [<DllImport(lib, EntryPoint = "lf_modifier_enable_vignetting_correction", CharSet = CharSet.Ansi); SuppressUnmanagedCodeSecurity>]
        extern int enableVignettingCorrection(LensFunModifierHandle handle, float32 aperture, float32 distance)

        [<DllImport(lib, EntryPoint = "lf_modifier_enable_projection_transform", CharSet = CharSet.Ansi); SuppressUnmanagedCodeSecurity>]
        extern int enableProjectionTransform(LensFunModifierHandle handle, LensType target)
        
        [<DllImport(lib, EntryPoint = "lf_modifier_enable_perspective_correction", CharSet = CharSet.Ansi); SuppressUnmanagedCodeSecurity>]
        extern int enablePerspectiveCorrection(LensFunModifierHandle handle, float32* x, float32* y, int count, float32 d)

        [<DllImport(lib, EntryPoint = "lf_modifier_get_auto_scale", CharSet = CharSet.Ansi); SuppressUnmanagedCodeSecurity>]
        extern float32 getAutoScale(LensFunModifierHandle handle, bool reverse)
        
        [<DllImport(lib, EntryPoint = "lf_modifier_apply_geometry_distortion", CharSet = CharSet.Ansi); SuppressUnmanagedCodeSecurity>]
        extern int applyGeometryDistortion(LensFunModifierHandle handle, float32 xu, float32 yu, int width, int height, V2f* res)
        
        [<DllImport(lib, EntryPoint = "lf_modifier_apply_color_modification", CharSet = CharSet.Ansi); SuppressUnmanagedCodeSecurity>]
        extern int applyColorModification(LensFunModifierHandle handle, nativeint pixels, float32 xu, float32 yu, int width, int height, int comp_role, int row_stride)

    // [<AbstractClass; NativeApi(Convention = CallingConvention.Cdecl)>]
    // type LensFun(ctx : Silk.NET.Core.Contexts.INativeContext) =
    //     inherit NativeAPI(ctx)

    //     [<NativeApi(EntryPoint = "lf_db_new")>]
    //     abstract member  NewDatabase : unit -> LensFunDatabaseHandle

    //     [<NativeApi(EntryPoint = "lf_db_load")>]
    //     abstract member LoadDatabase : db : LensFunDatabaseHandle -> unit

    //     [<NativeApi(EntryPoint = "lf_db_destroy")>]
    //     abstract member DestroyDatabase : db : LensFunDatabaseHandle -> unit
    
    //     [<NativeApi(EntryPoint = "lf_db_load_file")>]
    //     abstract member LoadDatabaseFile : db : LensFunDatabaseHandle * path : string -> LensFunError
        
    //     [<NativeApi(EntryPoint = "lf_db_load_data")>]
    //     abstract member LoadData : db : LensFunDatabaseHandle * [<MarshalAs(UnmanagedType.LPStr)>] errorContext : string * [<MarshalAs(UnmanagedType.LPStr)>] data : string * dataSize : unativeint -> LensFunError

    //     [<NativeApi(EntryPoint = "lf_db_get_cameras")>]
    //     abstract member GetCameras : db : LensFunDatabaseHandle -> nativeptr<nativeptr<LensFunCameraHandle>>
        
    //     [<NativeApi(EntryPoint = "lf_db_get_mounts")>]
    //     abstract member GetMounts : db : LensFunDatabaseHandle -> nativeptr<nativeptr<LensFunMountHandle>>
        
    //     [<NativeApi(EntryPoint = "lf_db_get_lenses")>]
    //     abstract member GetLenses : db : LensFunDatabaseHandle -> nativeptr<nativeptr<LensFunLensHandle>>
        
    //     [<NativeApi(EntryPoint = "lf_db_find_cameras")>]
    //     abstract member FindCameras : db : LensFunDatabaseHandle * maker : string * model : string -> nativeptr<nativeptr<LensFunCameraHandle>>

    //     [<NativeApi(EntryPoint = "lf_db_find_cameras_ext")>]
    //     abstract member FindCameras : db : LensFunDatabaseHandle * maker : string * model : string * flags : SearchFlags -> nativeptr<nativeptr<LensFunCameraHandle>>
        
    //     [<NativeApi(EntryPoint = "lf_db_find_lenses_hd")>]
    //     abstract member FindLenses : db : LensFunDatabaseHandle * camera : nativeptr<LensFunCameraHandle> * maker : string * lensModel : string * flags : SearchFlags -> nativeptr<nativeptr<LensFunLensHandle>>

    //     [<NativeApi(EntryPoint = "lf_db_find_mount")>]
    //     abstract member FindMount : db : LensFunDatabaseHandle * name : string -> nativeptr<LensFunMountHandle>

    //     [<NativeApi(EntryPoint = "lf_lens_get_mount_names")>]
    //     abstract member GetLensMountNames : db : LensFunDatabaseHandle * lens : nativeptr<LensFunLensHandle> -> nativeptr<nativeptr<byte>>

    //     [<NativeApi(EntryPoint = "lf_lens_interpolate_distortion")>]
    //     abstract member InterpolateDistortion : lens : nativeptr<LensFunLensHandle> * crop : float32 * focal : float32 * distortion : byref<LensCalibDistortionHandle> -> int


    //     [<NativeApi(EntryPoint = "lf_modifier_create")>]
    //     abstract member CreateModifier : lens : nativeptr<LensFunLensHandle> * focal : float32 * imgCrop : float32 * width : int * height : int * format : LensFunPixelFormat * reverse : bool -> LensFunModifierHandle
        
    //     [<NativeApi(EntryPoint = "lf_modifier_destroy")>]
    //     abstract member DestroyModifier : m : LensFunModifierHandle -> unit
        
    //     [<NativeApi(EntryPoint = "lf_modifier_enable_scaling")>]
    //     abstract member EnableScaling : m : LensFunModifierHandle * scale : float32 -> int
        
    //     [<NativeApi(EntryPoint = "lf_modifier_enable_distortion_correction")>]
    //     abstract member EnableDistortionCorrection : m : LensFunModifierHandle -> int
        
    //     [<NativeApi(EntryPoint = "lf_modifier_enable_tca_correction")>]
    //     abstract member EnableTcaCorrection : m : LensFunModifierHandle -> int
        
    //     [<NativeApi(EntryPoint = "lf_modifier_enable_vignetting_correction")>]
    //     abstract member EnableVignettingCorrection : m : LensFunModifierHandle * aperture : float32 * distance : float32 -> int
        
    //     [<NativeApi(EntryPoint = "lf_modifier_enable_projection_transform")>]
    //     abstract member EnableProjectionTransform : m : LensFunModifierHandle * target : LensType -> int
        
    //     [<NativeApi(EntryPoint = "lf_modifier_enable_perspective_correction")>]
    //     abstract member EnablePerspectiveCorrection : m : LensFunModifierHandle * x : nativeptr<float32> * y : nativeptr<float32> * count : int * d : float32 -> int

    //     [<NativeApi(EntryPoint = "lf_modifier_get_auto_scale")>]
    //     abstract member GetAutoScale : m : LensFunModifierHandle * reverse : bool -> float32
        
    //     [<NativeApi(EntryPoint = "lf_modifier_apply_geometry_distortion")>]
    //     abstract member ApplyGeometryDistortion : m : LensFunModifierHandle * xu : float32 * yu : float32 * width : int * height : int * res : nativeptr<V2f> -> int
        
    //     [<NativeApi(EntryPoint = "lf_modifier_apply_color_modification")>]
    //     abstract member ApplyColorModification : m : LensFunModifierHandle * pixels : nativeint * xu : float32 * yu : float32 * width : int * height : int * comp_role : int * row_stride : int -> int

type private PointerEnumerator<'a, 'b when 'a : unmanaged>(ptr : nativeptr<nativeptr<'a>>, read : nativeptr<'a> -> 'b) =
    let mutable current = ptr
    let mutable v = Unchecked.defaultof<'b>

    member x.MoveNext() =
        if current <> NativePtr.zero then
            let p = NativePtr.read current
            if p <> NativePtr.zero then 
                v <- read p
                current <- NativePtr.add current 1
                true
            else
                v <- Unchecked.defaultof<'b>
                false
        else
            v <- Unchecked.defaultof<'b>
            false

    member x.Current = v

    member x.Reset() =
        current <- ptr
        v <- Unchecked.defaultof<_>

    member x.Dispose() =
        current <- NativePtr.zero
        v <- Unchecked.defaultof<_>
        
    interface System.Collections.IEnumerator with
        member x.MoveNext() = x.MoveNext()
        member x.Current = x.Current :> obj
        member x.Reset() = x.Reset()

    interface System.Collections.Generic.IEnumerator<'b> with
        member x.Dispose() = x.Dispose()
        member x.Current = x.Current

type private PointerEnumerable<'a, 'b when 'a : unmanaged>(ptr : nativeptr<nativeptr<'a>>, read : nativeptr<'a> -> 'b) =
    
    interface System.Collections.IEnumerable with
        member x.GetEnumerator() = new PointerEnumerator<_,_>(ptr, read) :> System.Collections.IEnumerator
        
    interface System.Collections.Generic.IEnumerable<'b> with
        member x.GetEnumerator() = new PointerEnumerator<_,_>(ptr, read) :> System.Collections.Generic.IEnumerator<'b>

type ImageInfo =
    { 
        size        : V2i
        focalLength : float
        aperture    : option<float>
        distance    : option<float>
    }

type private Loader() =
    inherit LibraryLoader()

    override x.CoreLoadNativeLibrary(name : string) : nativeint =
        
        let ptr = Aardvark.LoadLibrary(typeof<Loader>.Assembly, name)
        ptr

    override x.CoreFreeNativeLibrary(name : nativeint) : unit =
        ()

    override x.CoreLoadFunctionPointer(library : nativeint, name : string) =
        Aardvark.GetProcAddress(library, name)


type LensFunDatabase private() =
    static let path = 
        Path.Combine(Environment.GetFolderPath Environment.SpecialFolder.LocalApplicationData, "LensFunSharp", "database")

    
    static let download() =
        
        use wc = new WebClient()
        let data = wc.DownloadData "https://github.com/lensfun/lensfun/archive/master.zip"
        use ms = new MemoryStream(data)
        use arch = new ZipArchive(ms)
        if not (Directory.Exists path) then Directory.CreateDirectory path |> ignore

        for e in arch.Entries do
            let ext = Path.GetExtension(e.Name).ToLower()
            if ext = ".xml" && e.FullName.StartsWith "lensfun-master/data/db/" then
                let dst = Path.Combine(path, e.Name)
                e.ExtractToFile dst

    static let files =
        lazy (
            if not (Directory.Exists path) then download()
            Directory.GetFiles(path, "*.xml")

        )

    static member Files = files.Value




type LensFun(databaseFiles : string[]) =
    let mutable db = Implementation.LensFunRaw.newDatabase()
    let dbFiles = System.Collections.Generic.HashSet<string>()

    static let camCache    = System.Collections.Concurrent.ConcurrentDictionary<LensFun * nativeptr<Implementation.LensFunCameraHandle>, Camera>()
    static let mountCache  = System.Collections.Concurrent.ConcurrentDictionary<LensFun * nativeptr<Implementation.LensFunMountHandle>, Mount>()
    static let lensCache   = System.Collections.Concurrent.ConcurrentDictionary<LensFun * nativeptr<Implementation.LensFunLensHandle>, Lens>()

    static let getCam self ptr = camCache.GetOrAdd((self, ptr), fun (self, ptr) -> Camera(self, ptr))
    static let getMount self ptr = mountCache.GetOrAdd((self, ptr), fun (self, ptr) -> Mount(self, ptr))
    static let getLens self ptr = lensCache.GetOrAdd((self, ptr), fun (self, ptr) -> Lens(self, ptr))

    static let getCameras (self : LensFun) (ptr : nativeptr<nativeptr<Implementation.LensFunCameraHandle>>) =
        PointerEnumerable(ptr, getCam self) :> seq<_>
                
    static let getMounts (self : LensFun) (ptr : nativeptr<nativeptr<Implementation.LensFunMountHandle>>) =
        PointerEnumerable(ptr, getMount self) :> seq<_>

    static let getLenses (self : LensFun) (ptr : nativeptr<nativeptr<Implementation.LensFunLensHandle>>) =
        PointerEnumerable(ptr, getLens self) :> seq<_>

    do for f in databaseFiles do
        if dbFiles.Add f then
            match Implementation.LensFunRaw.loadDatabaseFile(db, f) with
            | Implementation.LensFunError.NoError -> ()
            | err -> 
                dbFiles.Remove f |> ignore
                printfn "could not load %s: %A" (Path.GetFileName f) err
        
    member internal x.DB = db

    member x.DatabaseFiles = dbFiles :> seq<_>

    member x.Handle = db.Handle


    member x.AddFile (file : string) =
        if File.Exists file then 
            let err = Implementation.LensFunRaw.loadDatabaseFile(db, file)
            err = Implementation.LensFunError.NoError
        else
            false

    member x.AddXml(xml : string) =
        let err = Implementation.LensFunRaw.loadData(db, "xml", xml, nativeint xml.Length)
        err = Implementation.LensFunError.NoError

    member x.GetCameras() =
        if db.IsNull then raise <| ObjectDisposedException("LensFun")
        let ptr = Implementation.LensFunRaw.getCameras(db)
        getCameras x ptr
        
    member x.GetMounts() =
        if db.IsNull then raise <| ObjectDisposedException("LensFun")
        let ptr = Implementation.LensFunRaw.getMounts(db)
        getMounts x ptr

    member x.GetLenses() =
        if db.IsNull then raise <| ObjectDisposedException("LensFun")
        let ptr = Implementation.LensFunRaw.getLenses(db)
        getLenses x ptr
        
       
    member x.FindCameras(make : string, model : string, ?flags : SearchFlags) =
        if db.IsNull then raise <| ObjectDisposedException("LensFun")

        let ptr =
            match flags with
            | Some flags -> Implementation.LensFunRaw.findCamerasExt(db, make, model, flags)
            | None -> Implementation.LensFunRaw.findCameras(db, make, model)
        
        getCameras x ptr

    member x.FindMount(name : string) =
        if db.IsNull then raise <| ObjectDisposedException("LensFun")
        let ptr = Implementation.LensFunRaw.findMount(db, name)
        if ptr = NativePtr.zero then None
        else Mount(x, ptr) |> Some
        
    member x.FindLenses(cam : Camera, ?make : string, ?model : string, ?flags : SearchFlags) =
        if db.IsNull then raise <| ObjectDisposedException("LensFun")
        let ptr = Implementation.LensFunRaw.findLenses(db, cam.Handle, defaultArg make null, defaultArg model null, defaultArg flags SearchFlags.None)
        getLenses x ptr

    member x.FindLensMounts(lens : Lens) =
        let pMounts = Implementation.LensFunRaw.getMountNames(db, lens.Handle)
        if pMounts = NativePtr.zero then
            Set.empty
        else
            let mutable res = Set.empty
            let mutable pArr = pMounts
            let mutable pStr = NativePtr.read pArr
            while pStr <> NativePtr.zero do
                let str = Marshal.PtrToStringAnsi (NativePtr.toNativeInt pStr)
                if not (isNull str) then res <- Set.add str res
                pArr <- NativePtr.add pArr 1
                pStr <- NativePtr.read pArr
            res

    member x.InterpolateDistortion(lens : Lens, cropFactor : float, focal : float) =
        let mutable res = Unchecked.defaultof<_>
        if Implementation.LensFunRaw.interpolateDistortion(lens.Handle, float32 cropFactor, float32 focal, &res) <> 0 then
            Some (Distortion(lens, res))
        else
            None

    member private x.Dispose(disposing : bool) =
        if disposing then GC.SuppressFinalize x
        if db.IsValid then
            Implementation.LensFunRaw.destroyDatabase db
            db <- Implementation.LensFunDatabaseHandle.Null

    member x.Dispose() = x.Dispose true
    override x.Finalize() = x.Dispose false

    interface IDisposable with
        member x.Dispose() = x.Dispose()

    new() = new LensFun(LensFunDatabase.Files)

    new(directory : string) =
        let files =
            if Directory.Exists directory then Directory.GetFiles(directory, "*.xml", SearchOption.AllDirectories)
            else [||]
        new LensFun(files)

and Camera internal(parent : LensFun, ptr : nativeptr<Implementation.LensFunCameraHandle>) =
    let handle = NativePtr.read ptr
    let maker = Marshal.PtrToStringAnsi(NativePtr.toNativeInt handle.Maker)
    let model = Marshal.PtrToStringAnsi(NativePtr.toNativeInt handle.Model)
    let variant = 
        if handle.Variant = NativePtr.zero then None
        else Marshal.PtrToStringAnsi(NativePtr.toNativeInt handle.Variant) |> Some
    let mountName = 
        if handle.Mount = NativePtr.zero then None
        else Marshal.PtrToStringAnsi(NativePtr.toNativeInt handle.Mount) |> Some

    let mount =
        mountName |> Option.bind parent.FindMount

    let crop = float handle.CropFactor
    let score = handle.Score

    member x.Handle = ptr
    member x.Maker = maker
    member x.Model = model
    member x.Variant = variant
    member x.MountName = mountName
    member x.Mount = mount
    member x.CropFactor = crop
    member x.Score = score

    override x.GetHashCode() = Unchecked.hash ptr
    override x.Equals o =
        match o with
        | :? Camera as o -> ptr = o.Handle
        | _ -> false

    override x.ToString() =
        sprintf "Camera { Make: %s, Model: %s, CropFactor: %g, Score: %d }" maker model crop score

    interface IComparable with
        member x.CompareTo o =
            match o with
            | :? Camera as o -> compare (NativePtr.toNativeInt ptr) (NativePtr.toNativeInt o.Handle)
            | _ -> 0

and Mount internal(parent : LensFun, ptr : nativeptr<Implementation.LensFunMountHandle>) =
    let handle = NativePtr.read ptr
    let name = Marshal.PtrToStringAnsi(NativePtr.toNativeInt handle.Name)
    let compat = 
        if handle.Compat = NativePtr.zero then
            Set.empty
        else
            let mutable s = Set.empty
            let mutable p = handle.Compat
            let mutable v = NativePtr.read p
            while v <> NativePtr.zero do
                s <- Set.add (Marshal.PtrToStringAnsi (NativePtr.toNativeInt v)) s
                p <- NativePtr.add p 1
                v <- NativePtr.read p
            s

    member x.Handle = ptr
    member x.Name = name
    member x.Compatible = compat
    
    override x.GetHashCode() = Unchecked.hash ptr
    override x.Equals o =
        match o with
        | :? Mount as o -> ptr = o.Handle
        | _ -> false

    override x.ToString() =
        sprintf "Mount { Name: %s, Compatible: { %s } }" name (String.concat ", " compat)

    interface IComparable with
        member x.CompareTo o =
            match o with
            | :? Mount as o -> compare (NativePtr.toNativeInt ptr) (NativePtr.toNativeInt o.Handle)
            | _ -> 0

and Lens internal(parent : LensFun, ptr : nativeptr<Implementation.LensFunLensHandle>) =
    let handle = NativePtr.read ptr
    let maker = Marshal.PtrToStringAnsi(NativePtr.toNativeInt handle.Maker)
    let model = Marshal.PtrToStringAnsi(NativePtr.toNativeInt handle.Model)

    member x.MinFocal = float handle.MinFocal
    member x.MaxFocal = float handle.MaxFocal
    member x.MinAperture = float handle.MinAperture
    member x.MaxAperture = float handle.MaxAperture
    member x.Type = handle.Type
    member x.CenterX = float handle.CenterX
    member x.CenterY = float handle.CenterY
    member x.Score = handle.Score
    member x.Handle = ptr
    member x.Make = maker
    member x.Model = model


    member x.CreateModifier(cam : Camera, info : ImageInfo, ?reverse : bool) =
        let reverse = defaultArg reverse false
        let lf = Implementation.LensFunRaw.createModifier(x.Handle, float32 info.focalLength, float32 cam.CropFactor, info.size.X,  info.size.Y, Implementation.LensFunPixelFormat.U8, reverse)
        let res = new Modifier(parent, cam, x, lf)
        
        res.EnableProjectionTransform(LensType.Rectilinear) |> ignore
        res.EnableDistortionCorrection() |> ignore
        res.EnableTcaCorrection() |> ignore
        match info.aperture, info.distance with
        | Some aperture, Some distance -> 
            res.EnableVignettingCorrection(aperture, distance) |> ignore
        | _ ->
            ()
        res.EnableScaling(res.GetAutoScale(reverse)) |> ignore
        res




    member x.TryGetRemapMatrix(cam : Camera, exifFocal : float, size : V2i, backward : bool) =
        let lf = Implementation.LensFunRaw.createModifier(x.Handle, float32 exifFocal, float32 cam.CropFactor, size.X, size.Y, Implementation.LensFunPixelFormat.F32, backward)
        use res = new Modifier(parent, cam, x, lf)

        res.EnableProjectionTransform(LensType.Rectilinear) |> ignore
        res.EnableDistortionCorrection() |> ignore
        let s = res.GetAutoScale(backward)
        res.EnableScaling (s * 0.9)|> ignore

        res.TryGetRemapMatrix(size)
        


    member x.InterpolateDistortion(cam : Camera, exifFocal : float) =
        parent.InterpolateDistortion(x, cam.CropFactor, exifFocal)

    override x.GetHashCode() = Unchecked.hash ptr
    override x.Equals o =
        match o with
        | :? Lens as o -> ptr = o.Handle
        | _ -> false

    override x.ToString() =
        sprintf "Lens { Make: %s, Model: %s, Type: %A, Focal: %g-%gmm }" maker model handle.Type handle.MinFocal handle.MaxFocal

    interface IComparable with
        member x.CompareTo o =
            match o with
            | :? Lens as o -> compare (NativePtr.toNativeInt ptr) (NativePtr.toNativeInt o.Handle)
            | _ -> 0

and [<RequireQualifiedAccess>] DistortionType =
    | None
    | Poly3 of k1 : float
    | Poly5 of k1 : float * k2 : float
    | PtLens of a : float * b : float * c : float
    | Acm of k1 : float * k2 : float * k3 : float * k4 : float * k5 : float

and Distortion internal(lens : Lens, data : Implementation.LensCalibDistortionHandle) =
    
    let kind =
        match data.Model with
        | DistortionModel.Poly3 ->
            DistortionType.Poly3(float data.P0)
        | DistortionModel.Poly5 ->
            DistortionType.Poly5(float data.P0, float data.P1)
        | DistortionModel.PtLens ->
            DistortionType.PtLens(float data.P0, float data.P1, float data.P2)
        | DistortionModel.Acm ->
            DistortionType.Acm(float data.P0, float data.P1, float data.P2, float data.P3, float data.P4)
        | _ ->
            DistortionType.None

    member x.Lens = lens
    member x.Info = kind
    member x.Distortion = kind
    member x.FocalLength = float data.RealFocal
    member x.Aspect = float data.Attributes.Aspect
    member x.CropFactor = float data.Attributes.CropFactor
    member x.CenterX = float data.Attributes.CenterX
    member x.CenterY = float data.Attributes.CenterY

    override x.ToString() =
        sprintf "Distortion { Info: %A, Focal: %g, CropFactor: %g }" kind data.RealFocal data.Attributes.CropFactor

and Modifier internal(parent : LensFun, cam : Camera, lens : Lens, handle : Implementation.LensFunModifierHandle) =
    
    static let a = 2
    static let i = 3
    static let r = 4
    static let g = 5
    static let b = 6

    static let cr2(a, b) =  a ||| (b <<< 4)
    static let cr3(a, b, c) = a ||| (b <<< 4) ||| (c <<< 8)
    static let cr4(a, b, c, d) = a ||| (b <<< 4) ||| (c <<< 8) ||| (d <<< 12)

    static let toCompRole (fmt : Col.Format) =
        match fmt with
        | Col.Format.RGB -> cr3(r, g, b)
        | Col.Format.RGBA -> cr4(r, g, b, a)
        | Col.Format.BGR -> cr3(b, g, r)
        | Col.Format.BGRA -> cr4(b, g, r, a)
        | _ -> failwithf "unknown format %A" fmt

    let remapMatrices = Dict<V2i, option<Matrix<V2f>>>()
    
    let getRemapMatrix (size : V2i) =
        remapMatrices.GetOrCreate(size, fun size ->
            let data = Array.zeroCreate (size.X * size.Y)

            let invSize = 1.0f / V2f size
            let gc = GCHandle.Alloc(data, GCHandleType.Pinned)
            let ret = Implementation.LensFunRaw.applyGeometryDistortion(handle, 0.0f, 0.0f, size.X, size.Y, NativePtr.ofNativeInt (gc.AddrOfPinnedObject()))
            gc.Free()
            if ret <> 0 then
                for i in 0 .. data.Length - 1 do
                    let tc : V2f = data.[i] * invSize
                    data.[i] <- tc
                Matrix<V2f>(data, 0L, V2l(size), V2l(1, size.X), V2l.Zero) |> Some
            else
                None
        )

    member x.Lens = lens
    member x.Handle = handle

    member x.Dispose() =
        Implementation.LensFunRaw.destroyModifier handle

    member x.EnableDistortionCorrection() =
        Implementation.LensFunRaw.enableDistortionCorrection(handle) <> 0
        
    member x.EnableProjectionTransform(dst : LensType) =
        Implementation.LensFunRaw.enableProjectionTransform(handle, dst) <> 0
        
    member x.EnableScaling(scale : float) =
        Implementation.LensFunRaw.enableScaling(handle, float32 scale) <> 0

    member x.EnableTcaCorrection() =
        Implementation.LensFunRaw.enableTcaCorrection(handle) <> 0
        
    member x.EnableVignettingCorrection(aperture : float, distance : float) =
        Implementation.LensFunRaw.enableVignettingCorrection(handle, float32 aperture, float32 distance) <> 0
        
    member x.GetAutoScale(backward : bool) =
        Implementation.LensFunRaw.getAutoScale(handle, backward) |> float

    member x.TryGetRemapMatrix(size : V2i) : option<Matrix<V2f>> =
        getRemapMatrix size

    member x.Apply(img : PixImage<byte>) =
        match getRemapMatrix img.Size with
        | Some mat ->
            let img = 
                match img.Format with
                | Col.Format.BGRA -> img
                | _ -> img.ToPixImage<byte>(Col.Format.BGRA)

            let res = PixImage<byte>(Col.Format.BGRA, img.Size)

            NativeMatrix.using mat (fun pCoords ->
                NativeVolume.using img.Volume (fun pSrc ->
                    let pSrcMat =
                        NativeMatrix<C4b>(
                            NativePtr.cast pSrc.Pointer,
                            MatrixInfo(
                                0L,
                                V2l(pSrc.SX, pSrc.SY),
                                V2l(pSrc.DX / 4L, pSrc.DY / 4L)
                            )
                        )
                    NativeVolume.using res.Volume (fun pDst ->
                        let pDstMat =
                            NativeMatrix<C4b>(
                                NativePtr.cast pDst.Pointer,
                                MatrixInfo(
                                    0L,
                                    V2l(pDst.SX, pDst.SY),
                                    V2l(pDst.DX / 4L, pDst.DY / 4L)
                                )
                            )

                        let lerp (t : float) (a : C4b) (b : C4b) =
                            lerp a b t

                        pCoords.CopyTo(pDstMat, fun v ->
                            pSrcMat.SampleLinear(V2d v, lerp)
                        )

                    )
                )
            )

            res
        | None ->
            img

            
    member x.Apply(img : PixImage) =
        match img with
        | :? PixImage<byte> as img -> x.Apply img
        | _ -> x.Apply (img.ToPixImage<byte>())

    interface IDisposable with
        member x.Dispose() = x.Dispose()




//module LensFunTest =
//    let run() =
//        let dllPath = @"C:\Users\Schorsch\Development\lensfun\install\bin\lensfun.dll"

//        let a = DynamicLinker.loadLibrary dllPath
//        Log.warn "%A" a.Handle
//        let dbs = Directory.GetFiles(@"C:\Users\Schorsch\Development\lensfun\data\db", "*.xml", SearchOption.AllDirectories)
//        use lf = new LensFun(dllPath, dbs)

        
//        let file = @"C:\Users\Schorsch\Desktop\Brusher\Datasets\orange\DSC02289.JPG"
//        let img = PixImageSharp.CreateImage file
//        let info = PixImageSharp.TryGetCameraInfo img
//        let img = img.ToPixImage().ToPixImage<byte>(Col.Format.BGRA)

//        match info with
//        | Some info ->

//            let cams = lf.FindCameras(info.make, info.model, LensFunSearchFlags.SortAndUniquify)

//            for c in cams do
//                Log.start "%s %s" c.Maker c.Model
//                Log.line "crop:   %g" c.CropFactor
//                Log.line "score:  %d" c.Score
//                c.Variant |> Option.iter (Log.line "variant: %s")
            
//                match c.Mount with
//                | Some m -> 
//                    let compat =
//                        if Set.isEmpty m.Compatible then ""
//                        else m.Compatible |> String.concat "; " |> sprintf " { %s }"
//                    Log.line "mount:  %s%s" m.Name compat
//                | None ->
//                    match c.MountName with
//                    | Some name ->
//                        Report.WarnNoPrefix(sprintf "mount %s (unknown)" name)
//                    | None ->
//                        ()
                    
//                    ()
//                let a = float info.size.NormMax / float info.size.NormMin
//                let chipDiag = 43.26661531 / c.CropFactor
//                let h = sqrt (sqr chipDiag / (1.0 + sqr a))
//                let w = a * h
//                Log.line "sensor: %.3gmm x %.3gmm" w h



//                Log.start "lenses"
//                for l in lf.FindLenses(c, ?make = info.lensMake, ?model = info.lensModel)  do
//                    Log.start "%s %s" l.Make l.Model
//                    Log.line "type:      %A" l.Type
//                    if l.MaxFocal <> l.MinFocal then
//                        Log.line "focal:     %g-%gmm" l.MinFocal l.MaxFocal
//                    else
//                        Log.line "focal:     %gmm" l.MinFocal

//                    if l.MaxAperture > l.MinAperture then
//                        Log.line "aperture:  %g-%g" l.MinAperture l.MaxAperture
//                    else
//                        Log.line "aperture:  %g" l.MinAperture
//                    Log.line "principal: %g %g" l.CenterX l.CenterY




//                    match l.InterpolateDistortion(c, info.focal) with
//                    | Some dist -> 
//                        Log.start "distortion"

//                        let a = dist.Aspect
//                        let chipDiag = 43.0 / dist.CropFactor
//                        let h = sqrt (sqr chipDiag / (1.0 + sqr a))
//                        let w = a * h

//                        let fovx = 2.0 * atan ((w / 2.0) / dist.FocalLength) * Constant.DegreesPerRadian
//                        let fovy = 2.0 * atan ((h / 2.0) / dist.FocalLength) * Constant.DegreesPerRadian


//                        // a^2 + f^2*a^2 = d^2
//                        // (1 + f^2) * a^2 = d^2


//                        Log.line "info:      %A" dist.Info
//                        Log.line "focal:     %gmm" dist.FocalLength
//                        Log.line "fov:       %.3g° %.3g°" fovx fovy
//                        Log.line "crop:      %A" dist.CropFactor
//                        Log.line "principal: %g %g" dist.CenterX dist.CenterY



//                        Log.stop()

                    

//                    | None ->
//                        ()

                    
//                    let name = l.Model.Replace(" ", "_").Replace("/", "-")
//                    use m = l.CreateModifier(c, { size = img.Size; focalLength = info.focal; aperture = None; distance = None })
//                    let res = m.Apply(img)
//                    res.SaveImageSharp (sprintf @"C:\Users\Schorsch\Desktop\Debug\u_%s.jpg" name)


//                    Log.stop()

//                Log.stop()

//                Log.stop()
//        | None ->
//            ()

//        //for c in lf.GetCameras() do
//        //    Log.start "%s %s" c.Maker c.Model
//        //    c.Variant |> Option.iter (Log.line "variant: %s")
//        //    c.Mount |> Option.iter (Log.line "mount:   %s")
//        //    Log.line "crop:    %g" c.CropFactor
//        //    Log.line "score:   %d" c.Score
//        //    Log.stop()

//        //printfn "%A" lf.Handle
