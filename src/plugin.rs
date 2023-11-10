use bevy::prelude::Plugin;
use bevy::{
    asset::io::embedded::EmbeddedAssetRegistry,
    prelude::*,
};
use std::path::{Path, PathBuf};

macro_rules! embedded_asset {
    ($embedded:ident, $path:expr) => {
        $embedded.insert_asset(
            PathBuf::new(),
            Path::new($path),
            include_bytes!(concat!("../assets/", $path)),
        );
    };
}

pub struct EmbeddedAssetsPlugin;

impl Plugin for EmbeddedAssetsPlugin {
    fn build(&self, app: &mut App) {
        let embedded = app.world.resource_mut::<EmbeddedAssetRegistry>();
        embedded_asset!(embedded, "character_controller_demo.glb");
    }
}
