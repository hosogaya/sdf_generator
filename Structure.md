# Sdf generator 
[voxfield](https://github.dev/VIS4ROB-lab/voxfield)を参考に作成している．

# Common Type 
* `AnyIndex`: 3次元のインデックス
* `AnyIndexHash`: AnyIndexに対するハッシュ値を返す構造体
* `AnyIndexHashMap<ValueType>`: AnyIndexに対するValueTypeの `std::unordered_map`を定義するもの．std::vectorで定義するよりも`AnyIndexHash`によるハッシュ値でのアクセスの方が早いためこれが実装されていると考えれれる．


# Structure
* `Voxel`: ３次元の立方体．これに距離だとか勾配が保存されている．
* `Block`: 大きめの立方体にVoxelを敷き詰めたもの．Voxelは一次元配列で定義されており，`z, y, x`の順番でループを回してアクセするような形となっている．
* `Layer`: Blockに対する`std::unordered_map`を持っている．
* `Interpolator`: Voxelでデータを保存しているので，ある位置の距離を取得するのにvoxelのデータをそのまま使用すると誤差が出てしまう．これは，距離が求められた位置に対して周辺Voxelの距離データの平均値を計算して誤差を減らす．


# Point Cloud Process
* 観測したポイントクラウドから周囲環境（サーフェース）の法線ベクトルを計算するために一度２次元画像に落とし込んでいる．こうすることで，観測データの位置関係が明らかになるから外積で法線ベクトルを計算できる．