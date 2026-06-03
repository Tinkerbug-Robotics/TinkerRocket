//
//  CachingTileOverlay.swift
//  TinkerRocketApp
//
//  Offline maps — Trial 1 (see docs/plans/offline-maps.md).
//
//  A basemap-replacing tile overlay with a read-through disk cache. Per tile:
//  serve from disk if present; otherwise fetch from the provider, persist, and
//  return; if offline and not cached, fail so MapKit leaves that tile blank
//  rather than hanging. Only built for cache-permissive sources (USGS) — the
//  Apple basemap cases never use an overlay.
//

import MapKit

final class CachingTileOverlay: MKTileOverlay {
    private let sourceKey: String
    private let cache = OfflineTileCache.shared

    /// Neutral "not downloaded / no imagery" tile — a faint hatch — returned on
    /// any miss so offline gaps (or zoom past the provider's max) read as
    /// intentional rather than a broken/blank map (canReplaceMapContent leaves
    /// nothing underneath).
    private static let placeholderTile: Data = {
        let size = CGSize(width: 256, height: 256)
        let img = UIGraphicsImageRenderer(size: size).image { ctx in
            UIColor(white: 0.93, alpha: 1).setFill()
            ctx.fill(CGRect(origin: .zero, size: size))
            let c = ctx.cgContext
            c.setStrokeColor(UIColor(white: 0.82, alpha: 1).cgColor)
            c.setLineWidth(1)
            var x: CGFloat = -256
            while x < 256 {
                c.move(to: CGPoint(x: x, y: 0))
                c.addLine(to: CGPoint(x: x + 256, y: 256))
                x += 16
            }
            c.strokePath()
        }
        return img.pngData() ?? Data()
    }()

    init(source: TileSource) {
        sourceKey = source.rawValue
        super.init(urlTemplate: source.urlTemplate)
        canReplaceMapContent = true
        minimumZ = 0
        maximumZ = 19
    }

    override func loadTile(at path: MKTileOverlayPath,
                           result: @escaping (Data?, Error?) -> Void) {
        let key = sourceKey
        let cache = self.cache

        if let data = cache.tileData(source: key, z: path.z, x: path.x, y: path.y) {
            result(data, nil)
            return
        }

        var request = URLRequest(url: url(forTilePath: path))
        request.setValue("TinkerRocketApp/1.0 (offline map cache)",
                         forHTTPHeaderField: "User-Agent")
        URLSession.shared.dataTask(with: request) { data, response, error in
            guard let data = data,
                  let http = response as? HTTPURLResponse, http.statusCode == 200 else {
                // Offline miss, 404 past max zoom, etc. → neutral placeholder.
                result(Self.placeholderTile, nil)
                return
            }
            cache.store(data, source: key, z: path.z, x: path.x, y: path.y)
            result(data, nil)
        }.resume()
    }
}
