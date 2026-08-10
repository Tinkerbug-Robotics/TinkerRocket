package com.tinkerbug.tinkerrocket.app

import androidx.compose.foundation.layout.Box
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Download
import androidx.compose.material.icons.filled.Explore
import androidx.compose.material.icons.filled.Layers
import androidx.compose.material.icons.filled.Map
import androidx.compose.material.icons.filled.Public
import androidx.compose.material.icons.filled.Satellite
import androidx.compose.material.icons.filled.Terrain
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.Icon
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.vector.ImageVector
import com.tinkerbug.tinkerrocket.app.theme.TrMapIconButton
import com.tinkerbug.tinkerrocket.app.theme.TrMenuRow
import com.tinkerbug.tinkerrocket.app.theme.TrPickerButton
import com.tinkerbug.tinkerrocket.maps.TileSource

/**
 * The basemap the map screen renders, and the picker that chooses it — the
 * iOS `Menu { Picker }` in MapView.swift, ported.
 *
 * Why a type above [TileSource]: the map offers one basemap the tile proxy
 * knows nothing about (OpenFreeMap Liberty, a keyless vector STYLE rather
 * than a raster source), so the selector's list and `:core:maps`' source
 * enum are deliberately not the same list.  Keeping OpenFreeMap out of
 * [TileSource] also keeps it out of `TileSource.entries`, which the proxy
 * and the region downloader iterate to build their template maps — a vector
 * style has no place in either.
 *
 * Ordering is the order the old cycling button advanced through, so anyone
 * with muscle memory for "tap twice for USGS Imagery" finds the same
 * sequence in the menu.
 */
internal enum class Basemap(val tileSource: TileSource?) {
    USGS_IMAGERY_TOPO(TileSource.USGS_IMAGERY_TOPO),
    USGS_TOPO(TileSource.USGS_TOPO),
    USGS_IMAGERY(TileSource.USGS_IMAGERY),
    GOOGLE_SATELLITE(TileSource.GOOGLE_SATELLITE),

    /** OpenFreeMap Liberty — keyless vector style, online-only, never cached. */
    OPENFREEMAP(null),
    ;

    /** Compass for the vector street map; the raster sources map in [pickerIcon]. */
    val icon: ImageVector get() = tileSource?.pickerIcon ?: Icons.Filled.Explore

    val label: String get() = tileSource?.pickerLabel ?: "OpenFreeMap (online)"

    /** Whether a saved offline area can cover this basemap. */
    val offlineCapable: Boolean get() = tileSource?.cacheable == true

    /**
     * Attribution shown while this basemap is active (provider terms).
     * [googleLive] is the Map Tiles API's live copyright string once the
     * proxy has a session; the static "© Google" stands in until then.
     */
    fun attribution(googleLive: String?): String = when (tileSource) {
        null -> OPENFREEMAP_ATTRIBUTION
        TileSource.GOOGLE_SATELLITE ->
            googleLive?.let { "Google · $it" } ?: TileSource.GOOGLE_ATTRIBUTION
        else -> TileSource.ATTRIBUTION
    }

    companion object {
        /**
         * The menu's rows.  Google Satellite needs a Map Tiles API key
         * compiled in; builds without one (every fork) must not offer a
         * source that can only fail.
         */
        fun options(googleAvailable: Boolean): List<Basemap> =
            entries.filter { googleAvailable || it != GOOGLE_SATELLITE }
    }
}

/**
 * Sources a saved offline area can cover, in the picker's order.  Derived
 * from `cacheable` rather than listed by hand, so a source added to
 * `:core:maps` lands in the right list by its own provider terms.
 */
internal val DOWNLOADABLE_SOURCES: List<TileSource> =
    Basemap.entries.mapNotNull { it.tileSource }.filter { it.cacheable }

/** OpenFreeMap Liberty — keyless vector style, online-only, never cached. */
internal const val OPENFREEMAP_STYLE_URI = "https://tiles.openfreemap.org/styles/liberty"
internal const val OPENFREEMAP_ATTRIBUTION =
    "OpenFreeMap · © OpenMapTiles · © OpenStreetMap contributors"

/**
 * Glyph per source — the iOS `TileSource.symbol` analog, mapped to the
 * nearest Material icon (mountain.2.fill → Terrain, globe.desk.fill →
 * Satellite, globe.americas.fill → Public).  Exhaustive by design: a new
 * source can't reach the picker without someone choosing its glyph.
 */
internal val TileSource.pickerIcon: ImageVector
    get() = when (this) {
        TileSource.USGS_IMAGERY_TOPO -> Icons.Filled.Terrain
        TileSource.USGS_TOPO -> Icons.Filled.Map
        TileSource.USGS_IMAGERY -> Icons.Filled.Satellite
        TileSource.GOOGLE_SATELLITE -> Icons.Filled.Public
    }

/**
 * Picker/plate label.  "(online)" marks the sources no saved area can cover,
 * which is the distinction that matters at a launch site with no signal —
 * and the one a cycling button could never show.  Same wording on iOS
 * (`TileSource.pickerLabel` in TileSource.swift).
 */
internal val TileSource.pickerLabel: String
    get() = if (cacheable) displayName else "$displayName (online)"

/**
 * Map-screen basemap picker: a layers glyph on a plate that opens the full
 * list with the active source checked (iOS MapView.swift's `Menu`).
 *
 * It replaces a cycling button that advanced one source per tap and showed
 * only the source it had landed on — four of five options invisible, which
 * is how a shipped Google Satellite basemap read as missing entirely
 * (Pixel 8, 2026-08-10).
 */
@Composable
internal fun BasemapMenuButton(
    selected: Basemap,
    options: List<Basemap>,
    onSelect: (Basemap) -> Unit,
    modifier: Modifier = Modifier,
    onManageOffline: (() -> Unit)? = null,
) {
    var open by remember { mutableStateOf(false) }
    Box(modifier) {
        TrMapIconButton(
            icon = Icons.Filled.Layers,
            // The button is a glyph, so the active source reaches TalkBack
            // here and the map's attribution plate carries it visually.
            contentDescription = "Map source — ${selected.label}",
            onClick = { open = true },
        )
        DropdownMenu(expanded = open, onDismissRequest = { open = false }) {
            options.forEach { option ->
                TrMenuRow(
                    label = option.label,
                    icon = option.icon,
                    selected = option == selected,
                    onClick = {
                        onSelect(option)
                        open = false
                    },
                )
            }
            // iOS keeps the offline-download entry in this same menu: it is
            // the answer to the "(online)" markers a row above.
            onManageOffline?.let { manage ->
                HorizontalDivider()
                DropdownMenuItem(
                    text = { Text("Manage offline maps…") },
                    leadingIcon = { Icon(Icons.Filled.Download, contentDescription = null) },
                    onClick = {
                        open = false
                        manage()
                    },
                )
            }
        }
    }
}

/**
 * Labeled source picker for the Save Area screen, which chooses among the
 * downloadable (cacheable) sources only.  Same rows as the map's menu; a
 * labeled plate instead of a glyph because this one sits in a header row,
 * not over imagery, and has the width for it.
 */
@Composable
internal fun TileSourcePickerButton(
    selected: TileSource,
    options: List<TileSource>,
    onSelect: (TileSource) -> Unit,
    modifier: Modifier = Modifier,
) {
    var open by remember { mutableStateOf(false) }
    Box(modifier) {
        TrPickerButton(
            label = selected.displayName,
            icon = selected.pickerIcon,
            onClick = { open = true },
        )
        DropdownMenu(expanded = open, onDismissRequest = { open = false }) {
            options.forEach { option ->
                TrMenuRow(
                    label = option.pickerLabel,
                    icon = option.pickerIcon,
                    selected = option == selected,
                    onClick = {
                        onSelect(option)
                        open = false
                    },
                )
            }
        }
    }
}
