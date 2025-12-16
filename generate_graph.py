import json
import math
import os
import random
from datetime import datetime
from typing import Dict, Any, Tuple

import networkx as nx
import numpy as np


SEED = 42


def _spring_layout(G: nx.Graph, n: int, seed: int) -> Dict[int, Tuple[float, float]]:
    # Spring layout tends to look good across many graph families
    k = 1.6 / math.sqrt(max(n, 2))
    return nx.spring_layout(G, seed=seed, k=k, iterations=250)


def _graph_stats(G: nx.Graph) -> Dict[str, Any]:
    n = G.number_of_nodes()
    m = G.number_of_edges()
    degs = [d for _, d in G.degree()]
    return {
        "nodes": n,
        "edges": m,
        "avg_degree": float(sum(degs) / max(n, 1)),
        "max_degree": int(max(degs) if degs else 0),
    }


def build_variant(variant: str, n: int, seed: int) -> Dict[str, Any]:
    rng = random.Random(seed)
    np_rng = np.random.default_rng(seed)

    # ----- Generate topology
    if variant == "small_world":
        # Watts–Strogatz
        k = 6 if n >= 60 else max(4, (n // 10) * 2)
        p = 0.12
        G = nx.watts_strogatz_graph(n, k, p, seed=seed)

    elif variant == "scale_free":
        # Barabási–Albert
        m = 3 if n >= 30 else 2
        G = nx.barabasi_albert_graph(n, m, seed=seed)

    elif variant == "community":
        # Stochastic block model: 3 communities with sparse bridges
        sizes = [n // 3, n // 3, n - 2 * (n // 3)]
        p_in = 0.18
        p_out = 0.012
        probs = [
            [p_in, p_out, p_out],
            [p_out, p_in, p_out],
            [p_out, p_out, p_in],
        ]
        G = nx.stochastic_block_model(sizes, probs, seed=seed)

        # store community label
        block = {}
        idx = 0
        for b, sz in enumerate(sizes):
            for _ in range(sz):
                block[idx] = b
                idx += 1
        nx.set_node_attributes(G, block, "community")

    elif variant == "spatial":
        # Random geometric graph in 2D space (proximity-based)
        pos = {i: (float(np_rng.random()), float(np_rng.random())) for i in range(n)}
        radius = 0.18 if n <= 200 else 0.14
        G = nx.random_geometric_graph(n, radius, pos=pos)

    elif variant == "grid":
        # 2D lattice-ish network (clean, corporate-friendly)
        side = int(math.sqrt(n))
        side = max(10, side)
        G = nx.grid_2d_graph(side, side)
        # relabel to 0..N-1
        mapping = {node: i for i, node in enumerate(G.nodes())}
        G = nx.relabel_nodes(G, mapping)

    elif variant == "hub_spoke":
        # One hub with rings of spokes: visually striking
        G = nx.star_graph(n - 1)
        # add some extra chord edges for interest
        extra = n // 3
        nodes = list(range(1, n))
        for _ in range(extra):
            u, v = rng.sample(nodes, 2)
            if not G.has_edge(u, v):
                G.add_edge(u, v)

    else:
        raise ValueError(f"Unknown variant: {variant}")

    # Ensure connected-ish for nicer spreading (if not connected, connect components lightly)
    if not nx.is_connected(G):
        comps = list(nx.connected_components(G))
        for i in range(len(comps) - 1):
            a = rng.choice(list(comps[i]))
            b = rng.choice(list(comps[i + 1]))
            G.add_edge(a, b)

    # ----- Layout
    pos = _spring_layout(G, n=G.number_of_nodes(), seed=seed)

    # ----- Centrality (compute once)
    # Betweenness can be expensive; use approximate for larger n
    if G.number_of_nodes() > 260:
        # sample-based approx
        k = 35
        btw = nx.betweenness_centrality(G, k=k, seed=seed, normalized=True)
    else:
        btw = nx.betweenness_centrality(G, normalized=True)

    # ----- Build JSON
    nodes = []
    for i in G.nodes():
        x, y = pos[i]
        base_size = 2.2 + 2.2 * rng.random()  # nice range for visual variety

        node_obj = {
            "key": str(i),
            "x": float(x),
            "y": float(y),
            "size": float(base_size),
            "label": f"Node {i}",
            "centrality": float(btw.get(i, 0.0)),
        }

        if variant == "community":
            node_obj["community"] = int(G.nodes[i].get("community", 0))
        else:
            node_obj["community"] = 0

        nodes.append(node_obj)

    edges = []
    for u, v in G.edges():
        edges.append({
            "key": f"{u}-{v}",
            "source": str(u),
            "target": str(v),
            "weight": float(0.25 + 0.75 * rng.random()),
            "type": variant,
        })

    return {
        "metadata": {
            "variant": variant,
            "seed": seed,
            "generated": datetime.now().isoformat(timespec="seconds"),
            "stats": _graph_stats(G),
        },
        "nodes": nodes,
        "edges": edges,
    }


def main():
    out_dir = "graphs"
    os.makedirs(out_dir, exist_ok=True)

    # Curated set of visually distinct variants (add more later easily)
    variants = [
        ("small_world", 280),
        ("scale_free", 260),
        ("community", 300),
        ("spatial", 240),
        ("grid", 256),
        ("hub_spoke", 220),
    ]

    manifest = {"generated": datetime.now().isoformat(timespec="seconds"), "variants": []}

    for i, (name, n) in enumerate(variants):
        # vary seed per variant so even same type isn’t identical if you expand later
        seed = SEED + 10 * i
        data = build_variant(name, n=n, seed=seed)
        filename = f"{name}.json"
        path = os.path.join(out_dir, filename)
        with open(path, "w") as f:
            json.dump(data, f)
        manifest["variants"].append({
            "id": name,
            "label": name.replace("_", " ").title(),
            "file": f"{out_dir}/{filename}",
            "nodes": data["metadata"]["stats"]["nodes"],
            "edges": data["metadata"]["stats"]["edges"],
        })
        print(f"Wrote {path}")

    with open("manifest.json", "w") as f:
        json.dump(manifest, f)
    print("Wrote manifest.json")


if __name__ == "__main__":
    main()

