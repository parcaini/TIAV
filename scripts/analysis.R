library(ggplot2)
library(readr)
library(reshape2)

ch_danger_1 <- read_csv(
  "../data/max_danger/CHN_Cho-2_2_I-1-1-multi-planner-simulation-result_max_danger_1.csv"
)

ch_danger_2 <- read_csv(
  "../data/max_danger/CHN_Cho-2_2_I-1-1-multi-planner-simulation-result_max_danger_2.csv"
)

ch_danger_3 <- read_csv(
  "../data/max_danger/CHN_Cho-2_2_I-1-1-multi-planner-simulation-result_max_danger_3.csv"
)

col_danger_1 <-
  read_csv(
    "../data/max_danger/DEU_Cologne-79_2_I-1-multi-planner-simulation-result_max_danger_1.csv"
  )

col_danger_2 <-
  read_csv(
    "../data/max_danger/DEU_Cologne-79_2_I-1-multi-planner-simulation-result_max_danger_2.csv"
  )

col_danger_3 <-
  read_csv(
    "../data/max_danger/DEU_Cologne-79_2_I-1-multi-planner-simulation-result_max_danger_3.csv"
  )

us15_danger_1 <-
  read_csv(
    "../data/max_danger/USA_US101-15_2_T-1-multi-planner-simulation-result_max_danger_1.csv"
  )

us15_danger_2 <-
  read_csv(
    "../data/max_danger/USA_US101-15_2_T-1-multi-planner-simulation-result_max_danger_2.csv"
  )

us15_danger_3 <-
  read_csv(
    "../data/max_danger/USA_US101-15_2_T-1-multi-planner-simulation-result_max_danger_3.csv"
  )

us23_danger_1 <-
  read_csv(
    "../data/max_danger/USA_US101-23_1_T-1-multi-planner-simulation-result_max_danger_1.csv"
  )

us23_danger_2 <-
  read_csv(
    "../data/max_danger/USA_US101-23_1_T-1-multi-planner-simulation-result_max_danger_2.csv"
  )

us23_danger_3 <-
  read_csv(
    "../data/max_danger/USA_US101-23_1_T-1-multi-planner-simulation-result_max_danger_3.csv"
  )

danger_all <-
  rbind(
    ch_danger_1,
    ch_danger_2,
    ch_danger_3,
    col_danger_1,
    col_danger_2,
    col_danger_3,
    us15_danger_1,
    us15_danger_2,
    us15_danger_3,
    us23_danger_1,
    us23_danger_2,
    us23_danger_3
  )

ch_random_1 <-
  read_csv(
    "../data/random/CHN_Cho-2_2_I-1-1-multi-planner-simulation-result_random_1.csv"
  )

ch_random_2 <-
  read_csv(
    "../data/random/CHN_Cho-2_2_I-1-1-multi-planner-simulation-result_random_2.csv"
  )

ch_random_3 <-
  read_csv(
    "../data/random/CHN_Cho-2_2_I-1-1-multi-planner-simulation-result_random_3.csv"
  )

col_random_1 <-
  read_csv(
    "../data/random/DEU_Cologne-79_2_I-1-multi-planner-simulation-result_random_1.csv"
  )

col_random_2 <-
  read_csv(
    "../data/random/DEU_Cologne-79_2_I-1-multi-planner-simulation-result_random_2.csv"
  )


col_random_3 <-
  read_csv(
    "../data/random/DEU_Cologne-79_2_I-1-multi-planner-simulation-result_random_3.csv"
  )

us15_random_1 <-
  read_csv(
    "../data/random/USA_US101-15_2_T-1-multi-planner-simulation-result_random_1.csv"
  )


us15_random_2 <-
  read_csv(
    "../data/random/USA_US101-15_2_T-1-multi-planner-simulation-result_random_2.csv"
  )



us15_random_3 <-
  read_csv(
    "../data/random/USA_US101-15_2_T-1-multi-planner-simulation-result_random_3.csv"
  )



us23_random_1 <-
  read_csv(
    "../data/random/USA_US101-23_1_T-1-multi-planner-simulation-result_random_1.csv"
  )


us23_random_2 <-
  read_csv(
    "../data/random/USA_US101-23_1_T-1-multi-planner-simulation-result_random_2.csv"
  )



us23_random_3 <-
  read_csv(
    "../data/random/USA_US101-23_1_T-1-multi-planner-simulation-result_random_3.csv"
  )


random_all <-
  rbind(
    ch_random_1,
    ch_random_2,
    ch_random_3,
    col_random_1,
    col_random_2,
    col_random_3,
    us15_random_1,
    us15_random_2,
    us15_random_3,
    us23_random_1,
    us23_random_2,
    us23_random_3
  )


danger_ch <- rbind(ch_danger_1, ch_danger_2, ch_danger_3)
danger_ch <-
  cbind(danger_ch, scenario = 'ch', algorithm = 'max_danger')

random_ch <- rbind(ch_random_1, ch_random_2, ch_random_3)
random_ch <- cbind(random_ch, scenario = 'ch', algorithm = 'random')

danger_col <- rbind(col_danger_1, col_danger_2, col_danger_3)
danger_col <-
  cbind(danger_col, scenario = 'col', algorithm = 'max_danger')

random_col <- rbind(col_random_1, col_random_2, col_random_3)
random_col <-
  cbind(random_col, scenario = 'col', algorithm = 'random')


danger_us15 <- rbind(us15_danger_1, us15_danger_2, us15_danger_3)
danger_us15 <-
  cbind(danger_us15, scenario = 'us15', algorithm = 'max_danger')

random_us15 <- rbind(us15_random_1, us15_random_2, us15_random_3)
random_us15 <-
  cbind(random_us15, scenario = 'us15', algorithm = 'random')


danger_us23 <- rbind(us23_danger_1, us23_danger_2, us23_danger_3)
danger_us23 <-
  cbind(danger_us23, scenario = 'us23', algorithm = 'max_danger')

random_us23 <- rbind(us23_random_1, us23_random_2, us23_random_3)
random_us23 <-
  cbind(random_us23, scenario = 'us23', algorithm = 'random')


all_ch <- rbind(danger_ch, random_ch)
all_col <- rbind(danger_col, random_col)
all_us15 <- rbind(danger_us15, random_us15)
all_us23 <- rbind(danger_us23, random_us23)

all_danger <- rbind(danger_ch, danger_col, danger_us15, danger_us23)
all_random <- rbind(random_ch, random_col, random_us15, random_us23)
all_data <- rbind(all_danger, all_random)


# compute effect sizes
danger_random_emergency_rank_sum <-
  sum(rank(
    c(
      all_danger$emergency_maneuvers,
      all_random$emergency_maneuvers
    )
  )[seq_along(all_danger$emergency_maneuvers)])
a12_danger_random_emergency <-
  (danger_random_emergency_rank_sum / nrow(all_danger) - (nrow(all_danger) + 1) / 2) / nrow(all_random)


# compute p-value
p_danger_random_emergency <- wilcox.test(all_danger$emergency_maneuvers, all_random$emergency_maneuvers)

# plot data
boxplot_algorithms_ch_emergency <- boxplot(all_ch$emergency_maneuvers ~ all_ch$algorithm)
boxplot_algorithms_ch_interactions <- boxplot(all_ch$interactions ~ all_ch$algorithm)
boxplot_algorithms_ch_goals_not_reached <- boxplot(all_ch$goals_not_reached ~ all_ch$algorithm)


# analysis for effects of the different algorithms
ggplot(all_data, aes(factor(algorithm), emergency_maneuvers, fill = scenario)) + stat_summary(fun = "mean", geom = "bar", position = "dodge") + scale_fill_brewer(palette = "Set1")
ggplot(all_data, aes(factor(algorithm), goals_not_reached, fill = scenario)) + stat_summary(fun = "mean", geom = "bar", position = "dodge") + scale_fill_brewer(palette = "Set1")
ggplot(all_data, aes(factor(algorithm), interactions, fill = scenario)) + stat_summary(fun = "mean", geom = "bar", position = "dodge") + scale_fill_brewer(palette = "Set1")
ggplot(all_data, aes(factor(algorithm), collisions, fill = scenario))  + stat_summary(fun = "mean", geom = "bar", position = "dodge") + scale_fill_brewer(palette = "Set1")


# analysis for effects of each scenario
avg_emergency_ch_danger <- mean(danger_ch$emergency_maneuvers)
avg_collisions_ch_danger <- mean(danger_ch$collisions)
avg_goals_not_reached_ch_danger <- mean(danger_ch$goals_not_reached)
avg_interactions_ch_danger <- mean(danger_ch$interactions)

avg_emergency_ch_random <- mean(random_ch$emergency_maneuvers)
avg_collisions_ch_random <- mean(random_ch$collisions)
avg_goals_not_reached_ch_random <- mean(random_ch$goals_not_reached)
avg_interactions_ch_random <- mean(random_ch$interactions)

average_values_ch <- c(avg_emergency_ch_danger,
                       avg_emergency_ch_random,
                       avg_collisions_ch_danger,
                       avg_collisions_ch_random,
                       avg_goals_not_reached_ch_danger,
                       avg_goals_not_reached_ch_random,
                       avg_interactions_ch_danger,
                       avg_interactions_ch_random)
algorithm_labels_ch <- c("max_danger",
                         "random",
                         "max_danger",
                         "random",
                         "max_danger",
                         "random",
                         "max_danger",
                         "random")
variable_labels_ch <- c("emergency_maneuvers",
                        "emergency_maneuvers",
                        "collisions",
                        "collisions",
                        "goals_not_reached",
                        "goals_not_reached",
                        "interactions",
                        "interactions")
avg_labeled_ch <- data.frame(average_values_ch, algorithm_labels_ch, variable_labels_ch)

ggplot(avg_labeled_ch, aes(x=algorithm_labels_ch, y=average_values_ch, fill = variable_labels_ch)) +
  geom_bar(stat="identity", position = "dodge") + scale_fill_brewer(palette = "Set1") + scale_y_sqrt()


avg_emergency_col_danger <- mean(danger_col$emergency_maneuvers)
avg_collisions_col_danger <- mean(danger_col$collisions)
avg_goals_not_reached_col_danger <- mean(danger_col$goals_not_reached)
avg_interactions_col_danger <- mean(danger_col$interactions)

avg_emergency_col_random <- mean(random_col$emergency_maneuvers)
avg_collisions_col_random <- mean(random_col$collisions)
avg_goals_not_reached_col_random <- mean(random_col$goals_not_reached)
avg_interactions_col_random <- mean(random_col$interactions)

average_values_col <- c(avg_emergency_col_danger,
                        avg_emergency_col_random,
                        avg_collisions_col_danger,
                        avg_collisions_col_random,
                        avg_goals_not_reached_col_danger,
                        avg_goals_not_reached_col_random,
                        avg_interactions_col_danger,
                        avg_interactions_col_random)
algorithm_labels_col <- c("max_danger",
                          "random",
                          "max_danger",
                          "random",
                          "max_danger",
                          "random",
                          "max_danger",
                          "random")
variable_labels_col <- c("emergency_maneuvers",
                         "emergency_maneuvers",
                         "collisions",
                         "collisions",
                         "goals_not_reached",
                         "goals_not_reached",
                         "interactions",
                         "interactions")
avg_labeled_col <- data.frame(average_values_col, algorithm_labels_col, variable_labels_col)

ggplot(avg_labeled_col, aes(x=algorithm_labels_col, y=average_values_col, fill = variable_labels_col)) +
  geom_bar(stat="identity", position = "dodge") + scale_fill_brewer(palette = "Set1") + scale_y_sqrt()


avg_emergency_us15_danger <- mean(danger_us15$emergency_maneuvers)
avg_collisions_us15_danger <- mean(danger_us15$collisions)
avg_goals_not_reached_us15_danger <- mean(danger_us15$goals_not_reached)
avg_interactions_us15_danger <- mean(danger_us15$interactions)

avg_emergency_us15_random <- mean(random_us15$emergency_maneuvers)
avg_collisions_us15_random <- mean(random_us15$collisions)
avg_goals_not_reached_us15_random <- mean(random_us15$goals_not_reached)
avg_interactions_us15_random <- mean(random_us15$interactions)

average_values_us15 <- c(avg_emergency_us15_danger,
                         avg_emergency_us15_random,
                         avg_collisions_us15_danger,
                         avg_collisions_us15_random,
                         avg_goals_not_reached_us15_danger,
                         avg_goals_not_reached_us15_random,
                         avg_interactions_us15_danger,
                         avg_interactions_us15_random)
algorithm_labels_us15 <- c("max_danger",
                           "random",
                           "max_danger",
                           "random",
                           "max_danger",
                           "random",
                           "max_danger",
                           "random")
variable_labels_us15 <- c("emergency_maneuvers",
                          "emergency_maneuvers",
                          "collisions",
                          "collisions",
                          "goals_not_reached",
                          "goals_not_reached",
                          "interactions",
                          "interactions")
avg_labeled_us15 <- data.frame(average_values_us15, algorithm_labels_us15, variable_labels_us15)

ggplot(avg_labeled_us15, aes(x=algorithm_labels_us15, y=average_values_us15, fill = variable_labels_us15)) +
  geom_bar(stat="identity", position = "dodge") + scale_fill_brewer(palette = "Set1") + scale_y_sqrt()


avg_emergency_us23_danger <- mean(danger_us23$emergency_maneuvers)
avg_collisions_us23_danger <- mean(danger_us23$collisions)
avg_goals_not_reached_us23_danger <- mean(danger_us23$goals_not_reached)
avg_interactions_us23_danger <- mean(danger_us23$interactions)

avg_emergency_us23_random <- mean(random_us23$emergency_maneuvers)
avg_collisions_us23_random <- mean(random_us23$collisions)
avg_goals_not_reached_us23_random <- mean(random_us23$goals_not_reached)
avg_interactions_us23_random <- mean(random_us23$interactions)

average_values_us23 <- c(avg_emergency_us23_danger,
                         avg_emergency_us23_random,
                         avg_collisions_us23_danger,
                         avg_collisions_us23_random,
                         avg_goals_not_reached_us23_danger,
                         avg_goals_not_reached_us23_random,
                         avg_interactions_us23_danger,
                         avg_interactions_us23_random)
algorithm_labels_us23 <- c("max_danger",
                           "random",
                           "max_danger",
                           "random",
                           "max_danger",
                           "random",
                           "max_danger",
                           "random")
variable_labels_us23 <- c("emergency_maneuvers",
                          "emergency_maneuvers",
                          "collisions",
                          "collisions",
                          "goals_not_reached",
                          "goals_not_reached",
                          "interactions",
                          "interactions")
avg_labeled_us23 <- data.frame(average_values_us23, algorithm_labels_us23, variable_labels_us23)

ggplot(avg_labeled_us23, aes(x=algorithm_labels_us23, y=average_values_us23, fill = variable_labels_us23)) +
  geom_bar(stat="identity", position = "dodge") + scale_fill_brewer(palette = "Set1") + scale_y_sqrt()