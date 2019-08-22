library(bnlearn)
library(Rgraphviz)
library(gRain)

# Definine the skeleton of the graph
affordance.dag = empty.graph(nodes = c("tool_cubeness", "tool_symmetry", "tool_convexity", "tool_eccentricity",
                                       "obj_sphereness", "obj_cubeness", "obj_symmetry", "obj_eccentricity",
                                       "action", "effect_X", "effect_Y"))
# Definine the directions
amat(affordance.dag) = matrix(c(0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L, 1L, 1L,
                                0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L, 1L, 1L,
                                0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L, 1L, 1L,
                                0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L, 1L, 1L,
                                0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L, 1L, 1L,
                                0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L, 1L, 1L,
                                0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L, 1L, 1L,
                                0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L, 1L, 1L,
                                0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L, 1L, 1L,
                                0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L,
                                0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L),
                              byrow = TRUE, nrow = 11, ncol = 11, dimnames = list(nodes(affordance.dag), nodes(affordance.dag)))
# Plot the BN
pp <- graphviz.plot(affordance.dag)
nodeRenderInfo(pp) <- list(fontsize=c(20),
                          label=c("action"="Action",
                                  "effect_X"="Effect X",
                                  "effect_Y"="Effect Y",
                                  # "tool_sphereness"="Tool \nSphereness",
                                  "tool_cubeness"="Tool \nCubeness",
                                  "tool_symmetry"="Tool \nSymmetry",
                                  "tool_convexity"="Tool \nConvexity",
                                  "tool_eccentricity"="Tool \nEccentricity",
                                  "obj_sphereness"="Object \nSphereness",
                                  "obj_cubeness"="Object \nCubeness",
                                  "obj_symmetry"="Object \nSymmetry",
                                  # "obj_convexity"="Object \nConvexity",
                                  "obj_eccentricity"="Object \nEccentricity"
                          ))
renderGraph(pp)

# Load train data
wd <- "/home/yiklungpang/catkin_ws/src/affordance_experiment/bn"
train_data_path <- file.path(wd, "processed_data", "train_data.csv")
train_data <- read.csv(train_data_path, header = TRUE, sep = ",", quote = "\"", dec = ".", row.names = NULL)

# Compute CPT
bn.bayes <- bn.fit(affordance.dag, data = train_data, method = "bayes", iss=100)

# Load test data
test_data_path <- file.path(wd, "processed_data", "test_data.csv")
test_data <- read.csv(test_data_path, header = TRUE, sep = ",", quote = "\"", dec = ".", row.names = NULL)

# Effect list for computing distance
effect_bins <- c('VN','LN','NM','LP','VP')

# Compute random test scores
random_gambling_score <- 0.0
random_accuracy <- 0.0
random_rp <- 0.0
random_distance <- 0.0
for (i in 1:nrow(test_data)) {
  # Set evidence
  bn.bayes <- bn.fit(affordance.dag, data = train_data, method = "bayes", iss=100)
  junction <- compile(as.grain(bn.bayes))
  evidence <- setEvidence(junction, nodes = c("tool_cubeness", "tool_symmetry", "tool_convexity", "tool_eccentricity",
                                              "obj_sphereness", "obj_cubeness", "obj_symmetry", "obj_eccentricity",
                                              "action"), 
                          states = c(toString(test_data[i,1]),toString(test_data[i,2]),toString(test_data[i,3]),toString(test_data[i,4]),toString(test_data[i,5]),toString(test_data[i,6]),toString(test_data[i,7]),toString(test_data[i,8]),toString(test_data[i,9])))
  # Marginalize
  effect.cpt <- querygrain(evidence, nodes = c("effect_X", "effect_Y"),
                          type = "joint")
  # Randomize if multiple max prob
  max_idx <- which(effect.cpt==max(effect.cpt), arr.ind=TRUE)
  if (nrow(max_idx) > 1) {
    random_rp <- random_rp + 1
    max_idx <- max_idx[sample(1:nrow(max_idx),1),]
  }
  
  # Find prediction
  pred_x <- rownames(effect.cpt)[max_idx[1]]
  pred_y <- colnames(effect.cpt)[max_idx[2]]
  real_x <- toString(test_data[i,10])
  real_y <- toString(test_data[i,11])
  
  # Verify prediction
  if (pred_x == real_x & pred_y == real_y) {
    random_gambling_score <- random_gambling_score + 4.0
    random_accuracy <- random_accuracy + 1.0
  } else {
    random_gambling_score <- random_gambling_score - 1.0
  }
  # Compute distance
  random_distance <- random_distance + abs(grep(pred_x,effect_bins)-grep(real_x,effect_bins)) + abs(grep(pred_y,effect_bins)-grep(real_y,effect_bins))
}
# Compute scores
random_gambling_score <- random_gambling_score/(4.0*nrow(test_data))
random_accuracy <- random_accuracy/(nrow(test_data))
random_rp <- random_rp/(nrow(test_data))
random_distance <- random_distance/(nrow(test_data)*4.0*2.0)

# Load leave one out data
loo_data_path <- file.path(wd, "processed_data", "loo_data.csv")
loo_data <- read.csv(loo_data_path, header = TRUE, sep = ",", quote = "\"", dec = ".", row.names = NULL)

# Compute leave one out score
loo_gambling_score <- 0.0
loo_accuracy <- 0.0
loo_rp <- 0.0
loo_distance <- 0.0
for (i in 1:nrow(loo_data)) {
  # Set evidence
  bn.bayes <- bn.fit(affordance.dag, data = train_data, method = "bayes", iss=100)
  junction <- compile(as.grain(bn.bayes))
  evidence <- setEvidence(junction, nodes = c("tool_cubeness", "tool_symmetry", "tool_convexity", "tool_eccentricity",
                                              "obj_sphereness", "obj_cubeness", "obj_symmetry", "obj_eccentricity",
                                              "action"), 
                          states = c(toString(loo_data[i,1]),toString(loo_data[i,2]),toString(loo_data[i,3]),toString(loo_data[i,4]),toString(loo_data[i,5]),toString(loo_data[i,6]),toString(loo_data[i,7]),toString(loo_data[i,8]),toString(loo_data[i,9])))
  # Marginalize
  effect.cpt <- querygrain(evidence, nodes = c("effect_X", "effect_Y"),
                           type = "joint")
  # Randomize if multiple max prob
  max_idx <- which(effect.cpt==max(effect.cpt), arr.ind=TRUE)
  if (nrow(max_idx) > 1) {
    loo_rp <- loo_rp + 1
    max_idx <- max_idx[sample(1:nrow(max_idx),1),]
  }
  # Find prediction
  pred_x <- rownames(effect.cpt)[max_idx[1]]
  pred_y <- colnames(effect.cpt)[max_idx[2]]
  real_x <- toString(loo_data[i,10])
  real_y <- toString(loo_data[i,11])
  # Verify prediction
  if (pred_x == real_x & pred_y == real_y) {
    loo_gambling_score <- loo_gambling_score + 4.0
    loo_accuracy <- loo_accuracy + 1.0
  } else {
    loo_gambling_score <- loo_gambling_score - 1.0
  }
  # Compute distance
  loo_distance <- loo_distance + abs(grep(pred_x,effect_bins)-grep(real_x,effect_bins)) + abs(grep(pred_y,effect_bins)-grep(real_y,effect_bins))
  
}
# Compute scores
loo_gambling_score <- loo_gambling_score/(4.0*nrow(loo_data))
loo_accuracy <- loo_accuracy/(nrow(loo_data))
loo_rp <- loo_rp/(nrow(loo_data))
loo_distance <- loo_distance/(nrow(loo_data)*4.0*2.0)