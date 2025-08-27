# Perché questi pezzi di codice sono ottimizzati

Questa nota spiega **nel dettaglio** le scelte di progettazione e le micro‑ottimizzazioni presenti nelle tre funzioni mostrate:
- `proximity_iterative(...)`
- `euclideanClusterOptimized(...)`
- `ClusteringOptimized(...)`

L’obiettivo complessivo è eseguire il **clustering euclideo** su una nuvola di punti LiDAR in modo **robusto e veloce** riducendo copie inutili, allocazioni dinamiche ripetute e costi di ricorsione, e massimizzando la **località di memoria**.

---

## Principi generali usati

1. **Evitare la ricorsione** ⇒ usare uno stack iterativo per la visita (DFS).  
   - **Benefici**: niente rischi di stack overflow su cluster grandi; meno overhead di chiamate di funzione; comportamento più prevedibile per la CPU (branch predictor, cache).

2. **Riuso dei buffer** (`nearby_buffer`, `processed`, ecc.).  
   - **Benefici**: si riduce la pressione sull’allocatore, si evita frammentazione e si migliora la prevedibilità delle latenze.

3. **`reserve()` su `std::vector`** per dimensioni attese.  
   - **Benefici**: si minimizzano le riallocazioni e le copie/move durante la crescita del vettore (amortized O(1) ⇒ O(1) “più stretto”).

4. **Strutture dati compatte** (`std::array<float,3>` invece di `PointT` PCL nelle fasi hot).  
   - **Benefici**: meno banda di memoria, migliore locality, minore pressione su cache L1/L2, loop più veloci.

5. **Marcatura “visitato” anticipata**.  
   - **Benefici**: si evita di inserire più volte lo stesso indice nello stack/queue; si riduce lavoro ridondante.

6. **Move semantics** quando si inseriscono cluster completati nel vettore dei cluster.  
   - **Benefici**: si evita la copia dei vettori (potenzialmente grandi).

7. **Kd‑Tree per ricerche di vicinato**.  
   - **Benefici**: rispetto a un brute force O(N²), si passa a costi medi **O(log N + k)** per query (k = vicini trovati).

---

## `proximity_iterative(...)` — visita iterativa del cluster

Punti chiave di ottimizzazione:

- **Iterativa con `std::vector<int> stack`**  
  ```cpp
  std::vector<int> stack;
  stack.reserve(256);
  stack.push_back(start_idx);
  ```
  Niente ricorsione ⇒ meno overhead, nessun rischio di overflow su cluster grandi. `reserve(256)` è una **euristica** che copre molti casi pratici; in caso di crescita, il costo resta amortizzato.

- **Marcatura `processed[start_idx] = 1` immediata**  
  Marcando “visitato” quando si mette nello stack, si impedisce di pushare lo stesso nodo più volte, evitando inserimenti duplicati e lavoro superfluo.

- **`std::vector<char> processed`**  
  L’uso di `char` (1 byte) evita la specializzazione di `std::vector<bool>` che è un bit‑proxy meno “friendly” per la CPU (accessi non addressable per elemento). Qui ogni flag è un byte, **accesso O(1)** e prevedibile; ottimo per cache e semplicità.

- **Riuso del buffer dei vicini**  
  ```cpp
  std::vector<int>& nearby_buffer
  ...
  nearby_buffer = tree->search(...);
  ```
  Il buffer è **riutilizzato** tra iterazioni, perciò spesso **non rialloca**: l’assegnazione userà la capacità già presente quando sufficiente. È una riduzione netta delle allocazioni rispetto a creare ogni volta un nuovo `std::vector<int>`.

  > **Nota**: per massimizzare il beneficio, l’API di `KdTree::search` potrebbe accettare un buffer da **riempire in-place** (es. `void search(query, tol, std::vector<int>& out)`), così da **evitare del tutto** la costruzione del vettore temporaneo e possibili riallocazioni.

- **Accessi contigui ai dati**  
  La query alla KD‑tree usa direttamente i tre float contigui: `{points[current][0], ...}`. Meno indirection ⇒ migliore locality.

**Complessità**: ogni punto viene inserito nello stack al massimo una volta ⇒ **O(N + M)** dove M è il numero di archi effettivi (coppie entro `distanceTol`). Il costo dominante sta nelle chiamate a `search`, ottimizzate dalla KD‑tree.

---

## `euclideanClusterOptimized(...)` — orchestrazione delle visite

Punti chiave di ottimizzazione:

- **Preallocazione contenitori**  
  ```cpp
  clusters.reserve(64);
  std::vector<char> processed(n, 0);
  std::vector<int> nearby_buffer; nearby_buffer.reserve(64);
  ```
  - `clusters.reserve(64)`: meno riallocazioni quando i cluster sono numerosi.
  - `processed(n,0)`: una sola allocazione lineare, perfettamente cache‑friendly.
  - `nearby_buffer.reserve(64)`: dimensione tipica dei vicini per velocizzare sin da subito.

- **Salto immediato dei punti già processati**  
  ```cpp
  if (processed[i]) continue;
  ```
  Ogni punto è **visibile al massimo una volta**: si elimina lavoro ridondante.

- **Preallocazione del cluster**  
  ```cpp
  std::vector<int> cluster;
  cluster.reserve(256);
  ```
  Riduce le riallocazioni durante l’espansione; la scelta 256 è una euristica ragionevole per molti dataset LiDAR.

- **Move semantics quando si accoda il cluster**  
  ```cpp
  clusters.push_back(std::move(cluster));
  ```
  Evita la copia dell’intero vettore di indici (potenzialmente molto grande), riducendo tempo e banda di memoria.

**Complessità**: la logica garantisce che ogni indice venga espanso al più una volta ⇒ la parte non‑KD è **O(N)**. Il costo delle ricerche è **O(Q · log N + Σk)** con Q ≈ numero di espansioni (≤ N) e k vicini trovati.

---

## `ClusteringOptimized(...)` — pipeline completa PCL → compatto → KD → indici → PCL

Punti chiave di ottimizzazione:

- **Early exit**  
  ```cpp
  if (obsCloud->empty()) return clusters;
  ```
  Taglia i casi degeneri senza lavoro inutile.

- **Conversione a formato compatto**  
  ```cpp
  std::vector<std::array<float,3>> points;
  points.reserve(obsCloud->points.size());
  for (const auto& p : obsCloud->points)
      points.push_back({p.x, p.y, p.z});
  ```
  Passare da `PointT` (struttura PCL più “pesante”) a **tre `float` contigui**:
  - Riduce l’impronta in memoria durante le fasi hot (visite/ricerche).
  - Migliora la **località spaziale**: i tre float sono adiacenti e il vettore è contiguo.
  - Diminuisce i cache miss e la banda richiesta.

- **KD‑tree come membro della classe** (riuso intenzionale)  
  L’idea (commentata nel codice) è **riusare** la struttura ed evitare `new`/`delete` ad ogni chiamata. Nel frammento attuale viene ricreata; **idealmente** si dovrebbe implementare un `clear()/reset()` nella KD‑tree per **ricostruire in-place**.
  - **Perché è un’ottimizzazione**: la costruzione/distruzione ripetuta di strutture complesse può essere costosa; il riuso riduce allocazioni, frammentazione e riscalda le cache.

- **Inserimento nella KD‑tree**  
  ```cpp
  for (size_t i = 0; i < points.size(); ++i)
      this->tree_->insert({points[i][0], points[i][1], points[i][2]}, (int)i);
  ```
  Se la KD‑tree supporta un **bulk‑build** (costruzione da array), è spesso ancora più veloce (migliore bilanciamento + meno overhead di singole insert). Il codice è pronto per sfruttarlo in futuro.

- **Clustering sugli indici** (non sui punti PCL)  
  ```cpp
  auto idx_clusters = euclideanClusterOptimized(points, this->tree_, clusterTolerance);
  ```
  Lavorare su **indici** e su un array compatto evita copie di `PointT` e mantiene **hot** solo i dati essenziali (x,y,z).

- **Conversione finale con `reserve()` e filtri di taglia**  
  ```cpp
  if ((int)idxs.size() < minSize || (int)idxs.size() > maxSize) continue;
  cloud_cluster->points.reserve(idxs.size());
  for (int idx : idxs) cloud_cluster->points.push_back(obsCloud->points[idx]);
  ```
  - Si **filtrano** i cluster fuori range senza costi extra (nessuna creazione di cloud per cluster scartati).  
  - `reserve(idxs.size())` evita riallocazioni mentre si ricostruisce la `PointCloud` solo per i cluster che passano il filtro.

- **Misurazione del tempo con `steady_clock`**  
  Profilare con orologi stabili consente di **validare** le ottimizzazioni e prevenire regressioni.

**Complessità**:  
- **Build KD‑tree**: tipicamente **O(N log N)** (o **O(N)** se bulk‑build ottimizzato).  
- **Query**: ~ **O(log N + k)** per espansione (k = vicini entro `clusterTolerance`).  
- **Pipeline**: ~ **O(N log N + Σ (log N + k))**. Senza KD‑tree, un naive passerebbe a **O(N²)** nel caso peggiore.

---

## Impatto su cache, memoria e branch prediction

- **Cache locality**: `std::array<float,3>` in un `std::vector` è estremamente favorevole alla località. L’accesso sequenziale ai punti e agli indici del cluster riduce i cache miss.  
- **Dimensioni ridotte**: `float` invece di `double` dimezza la banda e spesso è sufficiente in ambito LiDAR.  
- **Branch prediction**: la condizione `if (!processed[id])` ha una distribuzione di esiti che tende a stabilizzarsi nei cluster compatti; il predictor ne beneficia.  
- **Allocazioni**: l’uso estensivo di `reserve()` e il riuso dei buffer limita la pressione sullo heap e la frammentazione.

---

## Invarianti di correttezza rilevanti per la performance

- **Processed marcato al push**: impedisce duplicati nello stack ⇒ evita lavoro quadratico.  
- **Visita per indici**: ogni indice è inserito al più una volta ⇒ linearizza la parte di visita.  
- **Filtri di taglia post‑cluster**: si evita di materializzare `PointCloud` per cluster scartati.

---

## Possibili **ulteriori** miglioramenti (safe & facili)

1. **API KD‑tree “fill‑into‑buffer”**  
   Cambiare la firma di `search` in:
   ```cpp
   void search(const float q[3], float tol, std::vector<int>& out);
   ```
   con `out.clear();` e `out.push_back(...)` interni. Così si **elimina** la creazione/assegnazione del vettore temporaneo e si **riusa** sempre la capacità già allocata.

2. **Riuso reale della KD‑tree**  
   Implementare `clear()`/`reset()` e ricostruire in‑place: **meno `new/delete`**, meno frammentazione. In C++, preferibile gestire `tree_` con **`std::unique_ptr<KdTree>`** per RAII e sicurezza.

3. **Bulk‑build** della KD‑tree** (se disponibile)**  
   Costruzione da array ordinato/mediani scelti ⇒ albero più bilanciato e tempi di query più stabili.

4. **Early‑filter su cluster troppo piccoli**  
   Si può interrompere l’espansione quando il cluster non potrà più raggiungere `minSize` (dipende dalla distribuzione dei punti e dalla politica di visita). Con attenzione, può ridurre il lavoro in scenari rumorosi.

5. **Parallelizzazione a livello di cluster seed**  
   Avviando più seed in parallelo (con attenzione a sincronizzazione su `processed`) o usando una pass a blocchi. Richiede design lock‑free o bitmap atomiche, ma scala bene su multi‑core.

6. **Layout dei dati ancora più cache‑friendly (SoA)**  
   Valutare *Structure of Arrays* (tre `std::vector<float>` per x,y,z) per massimizzare throughput SIMD nelle distanze. Dipende dalla vostra KD‑tree.

---

## TL;DR

- Si evita la **ricorsione** ⇒ meno overhead e nessun overflow.  
- Si **riusano buffer** e si **prealloca** la capacità ⇒ pochissime allocazioni runtime.  
- Si usa un **formato compatto** per i punti e una **KD‑tree** ⇒ si riduce banda di memoria e si ottiene complessità quasi‑lineare nella pratica.  
- Si usa **move semantics** e si limitano conversioni PCL solo a fine pipeline ⇒ niente copie costose inutili.

Il risultato è un **clustering più prevedibile, stabile e veloce**, soprattutto su nuvole di punti di grandi dimensioni.
