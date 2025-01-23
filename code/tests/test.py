import glob
import os
import unittest

from commonroad.common.file_reader import CommonRoadFileReader
from validation.validator import ScenarioValidator


class TestValidation(unittest.TestCase):

    def test_validation_good_scenarios(self):
        base_dir = "tests/good_scenarios"
        scenario_path = os.path.join(os.getcwd(), base_dir, "*.xml")
        files = glob.glob(scenario_path)

        validator = ScenarioValidator()

        for file in files:
            # load scenario
            crfr = CommonRoadFileReader(file)
            scenario, problem_set = crfr.open()
            scenario_name = file.split("/")[-1]
            print("Test scenario: ", scenario_name)
            self.assertTrue(validator.is_scenario_valid(
                scenario, problem_set), "Scenario should have been valid: " + scenario_name)

    def test_validation_bad_scenarios(self):
        base_dir = "tests/bad_scenarios"
        scenario_path = os.path.join(os.getcwd(), base_dir, "*.xml")
        files = glob.glob(scenario_path)

        validator = ScenarioValidator()

        for file in files:
            # load scenario
            crfr = CommonRoadFileReader(file)
            scenario, problem_set = crfr.open()
            scenario_name = file.split("/")[-1]
            print("Test scenario: ", scenario_name)
            self.assertFalse(validator.is_scenario_valid(
                scenario, problem_set), "Scenario should have been invalid: " + scenario_name)


if __name__ == '__main__':
    unittest.main()
