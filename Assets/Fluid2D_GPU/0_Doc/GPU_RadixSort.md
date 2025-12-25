# GPU Radix Sort
- GPU radix sort (1 bit version) in one pass
    1. get input
    2. get "bit" as b
    3. "inverse 'bit'" as E (if it is 0, count as 1)(bucket of 0!!!)
    4. Scan the 1s of E as F
    5. TotalFasle = e[last] + f[last] (total number of 0)
    6. T = i - F +  TotalFasle
    7. (address) d = b ? T:F
    8. Scatter by d

- Scan process
    1. UP Scan 
    2. DOWN Scan