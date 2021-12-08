import unittest
import gcp_finder_functions


class TestGcpFinder(unittest.TestCase):

    def test_get_border_scale(self):
        self.assertEqual(gcp_finder_functions.get_border_scale(50), 0.1464466094067262)
        self.assertEqual(gcp_finder_functions.get_border_scale(20), 0.05278640450004207)
        self.assertEqual(gcp_finder_functions.get_border_scale(1), 0.0025062814466900174)
        self.assertTrue(type(gcp_finder_functions.get_border_scale(1)) is (float or int))
        self.assertGreater(gcp_finder_functions.get_border_scale(1), 0)

    def test_imput_value_border_scale(self):
        self.assertRaises(TypeError, gcp_finder_functions.get_border_scale, True)


if __name__ == '__main__':
    unittest.main()

