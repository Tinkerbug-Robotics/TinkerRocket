//
//  OnboardingView.swift
//  TinkerRocketApp
//
//  First-launch onboarding: user picks a network name so their
//  devices don't collide with other users at the same field.
//

import SwiftUI

/// FNV-1a hash truncated to 1 byte — maps a network name to a network_id (0..255).
func fnv1a8(_ string: String) -> UInt8 {
    var hash: UInt32 = 2166136261  // FNV offset basis
    for byte in string.utf8 {
        hash ^= UInt32(byte)
        hash &*= 16777619  // FNV prime
    }
    // Fold 32-bit hash to 8 bits via XOR
    let folded = (hash ^ (hash >> 8) ^ (hash >> 16) ^ (hash >> 24)) & 0xFF
    return UInt8(folded)
}

struct OnboardingView: View {
    @AppStorage("networkName") private var networkName: String = ""
    @AppStorage("networkID") private var networkID: Int = -1
    @AppStorage("hasOnboarded") private var hasOnboarded: Bool = false

    @State private var nameInput: String = ""
    @State private var previewID: UInt8 = 0

    var body: some View {
        VStack(spacing: 32) {
            Spacer()

            Image("Small Tinkerbug Robotics Logo Vertical")
                .resizable()
                .aspectRatio(contentMode: .fit)
                .frame(height: 80)

            Text("Welcome to TinkerRocket")
                .font(.title)
                .fontWeight(.bold)

            Text("Choose a network name for your devices. This keeps your rockets and base stations separate from others at the same field.")
                .font(.body)
                .foregroundColor(.secondary)
                .multilineTextAlignment(.center)
                .padding(.horizontal, 32)

            VStack(alignment: .leading, spacing: 8) {
                Text("Network Name")
                    .font(.headline)

                TextField("e.g. My Backyard, Skyhawks Club", text: $nameInput)
                    .textFieldStyle(RoundedBorderTextFieldStyle())
                    .autocapitalization(.words)
                    .onChange(of: nameInput) { newValue in
                        if !newValue.isEmpty {
                            previewID = fnv1a8(newValue)
                        }
                    }

                if !nameInput.isEmpty {
                    Text("Network ID: \(previewID)")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
            }
            .padding(.horizontal, 32)

            Button {
                let trimmed = nameInput.trimmingCharacters(in: .whitespacesAndNewlines)
                guard !trimmed.isEmpty else { return }
                networkName = trimmed
                networkID = Int(fnv1a8(trimmed))
                hasOnboarded = true
            } label: {
                Text("Continue")
                    .fontWeight(.semibold)
                    .frame(maxWidth: .infinity)
                    .padding()
                    .background(nameInput.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty
                                ? Color.gray : Color.blue)
                    .foregroundColor(.white)
                    .cornerRadius(10)
            }
            .disabled(nameInput.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty)
            .padding(.horizontal, 32)

            Spacer()
        }
    }
}
