
```mermaid

flowchart TD
    %% スタイル定義（サイバー風・機能別色分け）
    classDef input fill:#1e3a8a,stroke:#60a5fa,color:#fff,stroke-width:2px
    classDef custom fill:#7c2d12,stroke:#f87171,color:#fff,stroke-width:2px
    classDef standard fill:#14532d,stroke:#4ade80,color:#fff,stroke-width:2px
    classDef output fill:#4b5563,stroke:#9ca3af,color:#fff,stroke-width:2px

    Input(["📡 LiDAR 生データ (/scan) <br>極座標 (距離 r, 角度 θ)"]) :::input

    subgraph PreProcess ["1. 前処理 (Pre-processing)"]
        direction TB
        ROI["✂️ 特化型ROIフィルタ (自作)<br>Z軸(石倉下)・XY軸(フィールド外)をごっそりカット"] :::custom
        Cartesian["📐 デカルト変換<br>(x = r*cosθ, y = r*sinθ)"] :::standard
    end

    subgraph CoreProcess ["2. 壁・平面抽出 (Core Processing)"]
        direction TB
        RANSAC["🎲 RANSAC (Open3D/PCL)<br>ランダムサンプリングで大まかな壁面を抽出"] :::standard
        TLS["🎯 TLS精密化 (自作・Eigen/NumPy)<br>特異値分解(SVD)による高精度な数式フィッティング"] :::custom
    end

    subgraph PostProcess ["3. 平滑化 (Post-processing)"]
        direction TB
        LPF["🌊 LPF [ローパスフィルタ] (自作)<br>過去データとブレンドし、角度のチャタリングを防止"] :::custom
    end

    Output(["📤 壁の角度・距離データ<br>(/wall_detection/angle, distance)"]) :::output

    %% フロー接続
    Input --> ROI
    ROI --> Cartesian
    Cartesian --> RANSAC
    RANSAC --> TLS
    TLS --> LPF
    LPF --> Output

```