use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use clap::Parser;
use garden_host::config::{Config, Paths};
use garden_host::engine::Engine;
use garden_host::{link, web};
use tracing_subscriber::EnvFilter;

/// The audio interface for the nrf53bird garden microphones: connects to the
/// USB receiver dongle and serves a web UI to listen, record and manage mics.
#[derive(Parser, Debug)]
#[command(version)]
struct Cli {
    /// Serial port of the receiver (default: found by its USB VID/PID).
    #[arg(long, value_name = "PATH")]
    port: Option<String>,
    /// Address for the web UI.
    #[arg(long, value_name = "ADDR", default_value = "127.0.0.1:8080")]
    listen: SocketAddr,
    /// Keep config.json and recordings/ here instead of the platform's
    /// config and data directories.
    #[arg(long, value_name = "DIR")]
    data_dir: Option<PathBuf>,
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .init();
    let cli = Cli::parse();
    let paths = match &cli.data_dir {
        Some(dir) => Paths::in_dir(dir),
        None => Paths::platform()?,
    };
    let cfg = Config::load(&paths.config_file)?;
    tracing::info!(
        "config {}, recordings in {}, {} known mic(s)",
        paths.config_file.display(),
        paths.recordings.display(),
        cfg.mics.len()
    );
    std::fs::create_dir_all(&paths.recordings)?;
    let engine = Arc::new(Mutex::new(Engine::new(
        cfg,
        paths,
        Box::new(Engine::local_now),
    )));
    link::spawn(engine.clone(), cli.port);

    let listener = tokio::net::TcpListener::bind(cli.listen).await?;
    tracing::info!("web UI on http://{}", listener.local_addr()?);
    let app = web::router(engine.clone());
    tokio::select! {
        r = axum::serve(listener, app) => r?,
        r = shutdown_signal() => {
            r?;
            tracing::info!("shutting down");
        }
    }
    // Completes the WAV headers of running recordings.
    link::lock(&engine).shutdown();
    Ok(())
}

/// Ctrl-C, or SIGTERM from a service manager.
async fn shutdown_signal() -> std::io::Result<()> {
    #[cfg(unix)]
    {
        let mut term = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())?;
        tokio::select! {
            r = tokio::signal::ctrl_c() => r,
            _ = term.recv() => Ok(()),
        }
    }
    #[cfg(not(unix))]
    tokio::signal::ctrl_c().await
}
