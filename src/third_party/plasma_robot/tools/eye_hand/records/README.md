# Validation records

This directory stores timestamped, reviewable hand-eye and TCP validation measurements.

`entry_reach_refinement_candidate_20260725.yaml` is a training-only source-frame correction
fitted from records 3-5. It is deliberately marked unvalidated and must not be used to enable
robot motion before independent blind validation.

`entry_reach_translation_candidate_20260726.yaml` is the translation-only comparison candidate.
It applies no rotation and remains preview-only. Its independent rounded-tip result is stored in
`entry_reach_translation_blind_20260726.yaml`: 21.660 mm total error, 19.498 mm lateral error,
and 11.709 deg axis error. The candidate failed and must not be loaded for robot motion.

`entry_reach_geometry_blind_20260726.yaml` preserves the first seventh-check observation, but it
is rejected because the then-current recorder confused the side-outlet entry TCP with the physical
rounded-tip mouth target. It must not be used for fitting or acceptance. Corrected software uses the
explicit `cavity_mouth_tip_pose` and writes subsequent checks to a new record file.

Generated reports must remain unapproved until their physical setup and independent path count
have been reviewed.
