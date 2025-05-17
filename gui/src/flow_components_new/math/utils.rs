pub fn gather<I, T, J>(values: I, indices: J) -> Vec<T>
where
    I: IntoIterator<Item = T> + Clone,
    T: Clone,
    J: IntoIterator<Item = usize>,
{
    let vec: Vec<T> = values.into_iter().collect();
    indices
        .into_iter()
        .map(|i| vec[i].clone())
        .collect()
}