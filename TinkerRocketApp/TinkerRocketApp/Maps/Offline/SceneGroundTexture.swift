import Foundation
import SceneKit
import UIKit

/// The satellite texture under a 3D SceneKit scene's ground plane, cached for
/// offline use (#838 item 1).
///
/// This existed twice — once in `DriftCastView` and once in
/// `FlightTrajectoryView`, byte-for-byte identical. When the app moved off Esri
/// World Imagery, only the Drift Cast copy was migrated. The saved-flight
/// trajectory kept hitting
/// `server.arcgisonline.com/.../World_Imagery/MapServer/export` with no cache
/// read and no cache write, so at a launch field with no signal the request
/// simply failed and the ground stayed a grey placeholder with a wireframe
/// grid — while the Drift Cast view for the same site rendered its cached
/// texture. With signal, every entry re-downloaded a 1024x1024 PNG the app
/// already had the machinery to cache.
///
/// Esri is not a licensing option here: `TileSource` records that its free
/// terms restrict offline caching, "which is the point of this feature". USGS
/// `USGSImageryOnly` is public domain and is what the 2D sources already use.
///
/// One copy, so the next migration cannot leave a view behind.
enum SceneGroundTexture {

    /// Public-domain USGS imagery export — no key, same source as the 2D
    /// offline tiles.
    static let endpoint =
        "https://basemap.nationalmap.gov/arcgis/rest/services/USGSImageryOnly/MapServer/export"

    /// The bounding box, request URL and cache key for a scene centred on
    /// `refLat`/`refLon`.
    ///
    /// Pure, so the endpoint and the key can be pinned by a test. The key is
    /// derived from the bbox alone, so two views showing the same site share
    /// one cached texture rather than each downloading its own.
    static func request(refLat: Double, refLon: Double,
                        extent: Float) -> (url: URL, cacheKey: String)? {
        let halfM = Double(extent * 1.5)
        let mPerDegLat = 110_540.0
        let mPerDegLon = 111_320.0 * cos(refLat * .pi / 180)
        guard mPerDegLat != 0, mPerDegLon != 0,
              halfM.isFinite, refLat.isFinite, refLon.isFinite else { return nil }

        let south = refLat - halfM / mPerDegLat
        let north = refLat + halfM / mPerDegLat
        let west = refLon - halfM / mPerDegLon
        let east = refLon + halfM / mPerDegLon

        let bbox = "\(west),\(south),\(east),\(north)"
        let urlStr = endpoint
            + "?bbox=\(bbox)&bboxSR=4326&imageSR=4326"
            + "&size=1024,1024&format=png&f=image"
        guard let url = URL(string: urlStr) else { return nil }
        return (url, "3d_\(bbox)")
    }

    /// Apply the ground texture, from cache when possible.
    ///
    /// Cache first, so a scene viewed once renders offline later. If there is
    /// no cached imagery and no connection the grid simply stays — the scene
    /// is still readable, it just has no terrain reference.
    static func apply(refLat: Double, refLon: Double, extent: Float,
                      groundNode: SCNNode, sceneRoot: SCNNode) {
        guard let req = request(refLat: refLat, refLon: refLon, extent: extent) else { return }

        let paint: (UIImage) -> Void = { image in
            DispatchQueue.main.async {
                groundNode.geometry?.firstMaterial?.diffuse.contents = image
                groundNode.geometry?.firstMaterial?.diffuse.wrapS = .clamp
                groundNode.geometry?.firstMaterial?.diffuse.wrapT = .clamp
                groundNode.geometry?.firstMaterial?.lightingModel = .constant
                // Remove the grid overlay now that we have imagery.
                sceneRoot.childNode(withName: "groundGrid", recursively: false)?
                    .removeFromParentNode()
            }
        }

        if let data = OfflineTileCache.shared.blob(forKey: req.cacheKey),
           let image = UIImage(data: data) {
            paint(image)
            return
        }

        URLSession.shared.dataTask(with: req.url) { data, response, _ in
            guard let data = data,
                  let httpResp = response as? HTTPURLResponse,
                  httpResp.statusCode == 200,
                  let image = UIImage(data: data) else { return }
            OfflineTileCache.shared.storeBlob(data, forKey: req.cacheKey)
            paint(image)
        }.resume()
    }
}
