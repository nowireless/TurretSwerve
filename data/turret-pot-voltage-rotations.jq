[inputs
| split("\t")
| {voltage: .[0], rotations: .[1]}
]
